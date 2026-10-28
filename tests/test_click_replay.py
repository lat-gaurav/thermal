"""Deterministic replay, cancellation, expiry and timing checks."""
import time
import math
import unittest
from unittest.mock import patch

import cv2
import numpy as np

import config
from flight.click_replay import ClickReplay, FrameHistory, EVENT_HEADER
from detector.boxtophat_scr import detect as real_detect
from experiment import los_static_track as los

Q = (1., 0., 0., 0.)


class Link:
    def attitude_span(self):
        return -100., 100., 100

    def q_at(self, t):
        return Q


def detect(image):
    n, labels, stats, centres = cv2.connectedComponentsWithStats((image > 200).astype(np.uint8))
    return [(x, y, w, h, 100.) for x, y, w, h, area in stats[1:]]


def populate(history, count=25, start=1, speed=2, missing=(), edge=False):
    for i in range(start, start+count):
        image = np.zeros((512, 640), np.uint8)
        x, y = (8, 8) if edge else (200+speed*i, 250)
        if i not in missing:
            image[y-5:y+5, x-5:x+5] = 250
        history.append(i, i/25., image)


class ReplayTest(unittest.TestCase):
    def setUp(self):
        self.history = FrameHistory()
        populate(self.history)
        self.worker = ClickReplay(self.history, Link(), 1516, 320, 256, 0.)
        self.addCleanup(self.worker.close)

    def wait_result(self, timeout=1.5):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            result = self.worker.take_live(self.history.latest_index, Q)
            state = self.worker.state()
            if result is not None or state['replay_status'] == 'failed':
                return result, state
            time.sleep(0.001)
        self.fail('replay did not finish')

    def submit(self, index=5, point=(210, 250), detector=detect):
        self.worker.submit(point, index, time.monotonic(), detector, [])

    def test_moving_target_replays_exact_clicked_frame_and_hands_over(self):
        self.submit()
        result, state = self.wait_result()
        self.assertIsNotNone(result)
        self.assertLess(abs(result[0]-250), 7)
        self.assertEqual(state['replay_status'], 'live')
        self.assertLessEqual(state['replay_remaining_frames'], config.CLICK_REPLAY_MAX_LAG_FRAMES)
        self.assertGreater(state['replay_steps'], 1)
        self.assertGreaterEqual(state['replay_click_to_live_ms'], state['replay_worker_ms'])
        self.assertIsNone(self.worker.take_live(25, Q))  # cannot hand over twice
        events = [dict(zip(EVENT_HEADER, row)) for row in self.worker.drain_events()]
        matched = next(e for e in events if e['event'] == 'historical_match')
        self.assertEqual(matched['replay_frame'], 5)
        self.assertEqual(events[-1]['event'], 'caught_up')

    def test_real_detector_on_native_scale_near_image_edge(self):
        self.history = FrameHistory()
        populate(self.history, edge=True)
        self.worker.history = self.history
        self.submit(point=(8, 8), detector=real_detect)
        result, state = self.wait_result()
        self.assertIsNotNone(result, state)
        self.assertLess(np.linalg.norm(np.array(result)-(8,8)), 4)

    def test_moving_camera_and_target(self):
        def attitude(t):
            angle = math.radians(t*25*.15)/2
            return (math.cos(angle), 0., 0., math.sin(angle))
        self.worker.link.q_at = attitude
        history = FrameHistory()
        positions = {}
        for i in range(1,26):
            uv = los.project((300.+i, 256.), Q, attitude(i/25), 1516,320,256)
            positions[i] = uv
            x,y = map(round,uv)
            image=np.zeros((512,640),np.uint8)
            image[y-5:y+5,x-5:x+5]=250
            history.append(i,i/25,image)
        self.worker.history=history
        self.worker.submit(positions[5],5,time.monotonic(),detect,[])
        deadline=time.monotonic()+1
        result=None
        while time.monotonic()<deadline and result is None:
            result=self.worker.take_live(25,attitude(1.))
            time.sleep(.001)
        self.assertIsNotNone(result,self.worker.state())
        self.assertLess(np.linalg.norm(np.array(result)-positions[25]),6)

    def test_capture_continues_while_replaying(self):
        import threading
        def delayed(image):
            time.sleep(.003)
            return detect(image)
        def append():
            time.sleep(.01)
            populate(self.history,count=3,start=26)
        producer=threading.Thread(target=append)
        producer.start()
        self.submit(detector=delayed)
        result,state=self.wait_result()
        producer.join()
        self.assertIsNotNone(result,state)
        self.assertGreaterEqual(state['replay_handover_frame'],26)

    def test_repeatable_handover_with_exact_frame_requirement(self):
        with patch.object(config,'CLICK_REPLAY_MAX_LAG_FRAMES',0):
            results=[]
            for _ in range(5):
                self.submit()
                result,state=self.wait_result()
                self.assertIsNotNone(result,state)
                results.append(result)
            self.assertTrue(all(np.allclose(results[0],r) for r in results))

    def test_unmatched_and_expired_clicks_fail_without_handover(self):
        for index, point in [(5, (500,100)), (999, (210,250))]:
            self.submit(index=index, point=point)
            result, state = self.wait_result()
            self.assertIsNone(result)
            self.assertEqual(state['replay_status'], 'failed')

    def test_missing_attitude_rejected_instead_of_clamped(self):
        self.worker.link.attitude_span = lambda: (10., 20., 100)
        self.submit()
        result, state = self.wait_result()
        self.assertIsNone(result)
        self.assertIn('attitude history', state['replay_detail'])

    def test_target_lost_rejects_candidate(self):
        self.history = FrameHistory()
        populate(self.history, missing=range(6,26))
        self.worker.history = self.history
        self.submit()
        result, state = self.wait_result()
        self.assertIsNone(result)
        self.assertIn('target lost', state['replay_detail'])

    def test_short_occlusion_recovers(self):
        self.history = FrameHistory()
        populate(self.history, missing=(11,12))
        self.worker.history = self.history
        self.submit()
        result, state = self.wait_result()
        self.assertIsNotNone(result, state)

    def test_timeout_does_not_handover(self):
        def slow(image):
            time.sleep(0.04)
            return detect(image)
        with patch.object(config, 'CLICK_REPLAY_DEADLINE_S', 0.02):
            self.submit(detector=slow)
            result, state = self.wait_result()
        self.assertIsNone(result)
        self.assertIn('deadline', state['replay_detail'])

    def test_new_click_supersedes_blocked_old_worker(self):
        import threading
        entered, release = threading.Event(), threading.Event()
        def blocked(image):
            entered.set()
            release.wait(1)
            return detect(image)
        self.submit(detector=blocked)
        self.assertTrue(entered.wait(1))
        self.submit(index=10, point=(220,250))
        release.set()
        result, state = self.wait_result()
        self.assertIsNotNone(result, state)
        self.assertEqual(state['replay_clicked_frame'], 10)

    def test_cancel_prevents_handover(self):
        self.submit()
        self.worker.cancel('algorithm off')
        self.assertIsNone(self.worker.take_live(25,Q))

    def test_history_is_bounded_and_keeps_original_image_references(self):
        history = FrameHistory(capacity=3)
        image = np.zeros((4,4), np.uint8)
        for i in range(10):
            history.append(i, i/25, image)
        self.assertIsNone(history.get(6))
        self.assertIs(history.get(7).image, image)
        self.assertEqual(len(history.frames), 3)


if __name__ == '__main__':
    unittest.main()
