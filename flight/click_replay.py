"""Bounded native-frame history and asynchronous historical-click replay.

The candidate never modifies the live tracker. Only take_live(), called by the
main loop, can hand over a confirmed candidate on a matching current frame.
"""
from collections import deque
from dataclasses import dataclass
import threading
import time

import config
from experiment import los_static_track as los
from initialisation.manual_click import init as manual_init


@dataclass(frozen=True)
class Frame:
    index: int
    t: float
    image: object


class FrameHistory:
    def __init__(self, capacity=None):
        self.frames = deque(maxlen=capacity or config.CLICK_HISTORY_FRAMES)
        self.lock = threading.Lock()

    def append(self, index, t, image):
        # Capture produces a new ndarray per read; retain ownership references,
        # not copies. Detectors and preview must treat these images as read-only.
        with self.lock:
            self.frames.append(Frame(index, t, image))

    def get(self, index):
        with self.lock:
            return next((f for f in self.frames if f.index == index), None)

    def next(self, index, stride):
        with self.lock:
            if not self.frames or self.frames[-1].index <= index:
                return None, None
            if self.frames[0].index > index + 1:
                raise ValueError('history expired during replay')
            first = next(f for f in self.frames if f.index > index)
            target = min(index + stride, self.frames[-1].index)
            step = next(f for f in self.frames if f.index >= target)
            return step, first if first.index < step.index else None

    @property
    def latest_index(self):
        with self.lock:
            return self.frames[-1].index if self.frames else 0


EVENT_HEADER = ['click_id', 'click_frame', 'click_x', 'click_y', 'event',
                'replay_frame', 'live_frame', 'remaining_frames', 'replay_steps',
                'click_elapsed_ms', 'worker_ms', 'match_distance_px', 'detail']


class ClickReplay:
    def __init__(self, history, link, focal, cx, cy, lookback):
        self.history, self.link = history, link
        self.focal, self.cx, self.cy, self.lookback = focal, cx, cy, lookback
        self.cv = threading.Condition()
        self.generation = 0
        self.job = None
        self.closed = False
        self.snapshots = deque(maxlen=8)
        self.events = deque(maxlen=128)
        self.info = dict(replay_status='idle', replay_detail='Click a target to acquire')
        self.thread = threading.Thread(target=self._run, name='click-replay', daemon=True)
        self.thread.start()

    def _event(self, event, detail='', live_frame=None):
        # Called with cv held. Event rows are drained/written by the main loop.
        now = time.monotonic()
        i = self.info
        self.events.append([
            i.get('replay_click_id', ''), i.get('replay_clicked_frame', ''),
            i.get('replay_click_x', ''), i.get('replay_click_y', ''), event,
            i.get('replay_frame', ''), live_frame if live_frame is not None else '',
            i.get('replay_remaining_frames', ''), i.get('replay_steps', 0),
            round(1000 * (now - i['received']), 2),
            round(1000 * (now - i['started']), 2), i.get('replay_match_distance_px', ''), detail])

    def submit(self, point, index, received, detect, filters):
        self.cancel('superseded by a newer click')
        with self.cv:
            self.generation += 1
            now = time.monotonic()
            self.info = dict(replay_status='past_frames', replay_detail='Detecting on clicked frame',
                             replay_click_id=self.generation, replay_clicked_frame=index,
                             replay_click_x=point[0], replay_click_y=point[1],
                             replay_frame=index, replay_steps=0,
                             replay_remaining_frames=max(0, self.history.latest_index-(index or 0)),
                             received=received, started=now)
            self.snapshots.clear()
            self.job = (self.generation, point, index, detect, tuple(filters), now)
            self._event('submitted')
            self.cv.notify_all()

    def cancel(self, reason):
        with self.cv:
            if self.info['replay_status'] in ('past_frames', 'catching_up', 'ready'):
                self._event('cancelled', reason)
                self.info.update(replay_status='cancelled', replay_detail=reason)
            elif self.info['replay_status'] == 'live' and reason == 'algorithm is off':
                self.info.update(replay_status='idle', replay_detail='Algorithm is off')
            self.generation += 1
            self.job = None
            self.snapshots.clear()
            self.cv.notify_all()

    def state(self):
        with self.cv:
            result = {k: v for k, v in self.info.items() if k not in ('received', 'started')}
            if self.info['replay_status'] in ('past_frames', 'catching_up', 'ready'):
                result['replay_elapsed_ms'] = round(1000*(time.monotonic()-self.info['received']), 1)
            return result

    def drain_events(self):
        with self.cv:
            events = list(self.events)
            self.events.clear()
            return events

    def _check(self, generation, started):
        with self.cv:
            if self.closed or generation != self.generation:
                raise InterruptedError()
        if time.monotonic()-started > config.CLICK_REPLAY_DEADLINE_S:
            raise TimeoutError('catch-up deadline exceeded; previous live lock retained')

    def _q(self, frame):
        t = frame.t-self.lookback
        oldest, newest, count = self.link.attitude_span()
        # q_at clamps; do not silently accept an uncovered historical timestamp.
        if count < 2 or oldest is None or newest is None or t < oldest or t > newest:
            raise ValueError('attitude history does not cover replay frame')
        return self.link.q_at(t)

    def _detect(self, frame, point, radius, detect, filters):
        h, w = frame.image.shape[:2]
        bounds = los.crop_bounds(*point, radius+config.CLICK_REPLAY_PADDING_PX, w, h)
        if bounds is None:
            return []
        x0, y0, x1, y1 = bounds
        boxes = [(x+x0, y+y0, bw, bh, score)
                 for x, y, bw, bh, score in detect(frame.image[y0:y1, x0:x1])]
        ctx = dict(frame_w=w, frame_h=h, los_point=None)
        for name, fn in filters:
            boxes = fn(boxes, ctx)
        return boxes

    def _publish(self, gen, frame, q, tracker, frame_idx, steps):
        with self.cv:
            if gen != self.generation:
                raise InterruptedError()
            remaining = max(0, self.history.latest_index-frame.index)
            self.info.update(replay_frame=frame.index, replay_steps=steps,
                             replay_remaining_frames=remaining,
                             replay_status='ready' if remaining <= config.CLICK_REPLAY_MAX_LAG_FRAMES else 'catching_up',
                             replay_detail='Waiting for live handover' if remaining <= config.CLICK_REPLAY_MAX_LAG_FRAMES
                             else 'Tracking through past frames')
            if tracker.status == 'tracking':
                self.snapshots.append((frame.index, tracker.point(frame_idx), q))

    def _replay(self, job):
        gen, point, index, detect, filters, started = job
        self._check(gen, started)
        frame = self.history.get(index)
        if frame is None:
            raise ValueError('clicked frame expired; click a fresh preview image')
        q = self._q(frame)
        boxes = self._detect(frame, point, config.MANUAL_CLICK_MAX_PX, detect, filters)
        self._check(gen, started)
        uv = manual_init(boxes, dict(manual_point=point,
                                    frame_w=frame.image.shape[1], frame_h=frame.image.shape[0]))
        if uv is None:
            raise ValueError('no detection near click on the clicked frame')
        qs, ts = [q], [frame.t]
        tracker = los.SmoothTracker(qs, ts, self.focal, self.cx, self.cy, los.project)
        tracker.set_click(0, uv)
        with self.cv:
            if gen != self.generation:
                raise InterruptedError()
            self.info['replay_match_distance_px'] = round(((uv[0]-point[0])**2+(uv[1]-point[1])**2)**0.5, 2)
            self._event('historical_match')
        steps = 0
        self._publish(gen, frame, q, tracker, 0, steps)
        while True:
            self._check(gen, started)
            step, intermediate = self.history.next(frame.index, config.CLICK_REPLAY_STRIDE)
            if step is None:
                with self.cv:
                    self.cv.wait(0.005)
                continue
            # Predict before appending. If a stride misses, roll back just that
            # update and try the intermediate frame, then the destination again.
            for target in ([step] if intermediate is None else [step, intermediate, step]):
                self._check(gen, started)
                q = self._q(target)
                qs.append(q); ts.append(target.t)
                i = len(ts)-1
                pred = tracker.point(i)
                if pred is None:
                    raise ValueError('candidate moved outside camera view')
                boxes = self._detect(target, pred, tracker.gate_px(i), detect, filters)
                self._check(gen, started)
                old_ref, old_misses = tracker.ref, tracker.misses
                tracker.update(i, boxes)
                steps += 1
                if target is step and intermediate is not None and tracker.status != 'tracking':
                    tracker.ref, tracker.misses = old_ref, old_misses
                    qs.pop(); ts.pop()
                    intermediate = None
                    continue
                if target is step:
                    break
                if tracker.misses > config.CLICK_REPLAY_MAX_MISSES:
                    raise ValueError('target lost during replay; previous live lock retained')
            if tracker.misses > config.CLICK_REPLAY_MAX_MISSES:
                raise ValueError('target lost during replay; previous live lock retained')
            frame = step
            self._publish(gen, frame, q, tracker, len(ts)-1, steps)

    def take_live(self, capture_index, current_q):
        """Return a live-frame position only when confirmed within the lag cap."""
        with self.cv:
            if self.info['replay_status'] not in ('ready', 'catching_up'):
                return None
            eligible = [s for s in self.snapshots if 0 <= capture_index-s[0] <= config.CLICK_REPLAY_MAX_LAG_FRAMES]
            if not eligible:
                return None
            frame, uv, q = eligible[-1]
            projected = los.project(uv, q, current_q, self.focal, self.cx, self.cy)
            if projected is None:
                return None
            now = time.monotonic()
            self.info.update(replay_status='live', replay_detail='Caught up; tracking live',
                             replay_frame=frame,
                             replay_handover_frame=capture_index,
                             replay_remaining_frames=capture_index-frame,
                             replay_click_to_live_ms=round(1000*(now-self.info['received']), 2),
                             replay_worker_ms=round(1000*(now-self.info['started']), 2))
            self.info['replay_detail'] = ('Historical match -> %d replay updates -> live in %.1f ms'
                                          % (self.info['replay_steps'], self.info['replay_click_to_live_ms']))
            self._event('caught_up', live_frame=capture_index)
            self.generation += 1
            self.snapshots.clear()
            return projected

    def _run(self):
        while True:
            with self.cv:
                self.cv.wait_for(lambda: self.closed or self.job is not None)
                if self.closed:
                    return
                job, self.job = self.job, None
            try:
                self._replay(job)
            except InterruptedError:
                pass
            except Exception as exc:
                with self.cv:
                    if job[0] == self.generation:
                        self.info.update(replay_status='failed', replay_detail=str(exc))
                        self._event('failed', str(exc))
                        self.snapshots.clear()

    def close(self):
        self.cancel('pipeline stopped')
        with self.cv:
            self.closed = True
            self.cv.notify_all()
        self.thread.join(timeout=2)
