"""Run something only while an RC switch is held ON, and fail safe when it isn't.

    algo = RcArm(config.ARM_RC_CHANNEL, config.ARM_RC_US)
    armed, reason = algo.update(link)      # once per frame

TWO INDEPENDENT SWITCHES, and they must stay independent. This rig flies with

    ch7  ALGO   -- run detection, tracking and the uplink
    ch6  RECORD -- write a .rawrec episode

so that all four combinations work: pipeline without recording, recording
without pipeline, both, or neither. Recording in particular must not depend on
anything the tracker produces -- with the algorithm off there is still a camera
feed worth capturing, and that footage is how the algorithm gets improved.

DISARMED IS THE SAFE STATE, so anything unknown disarms:

  * no RC packet ever seen            -> OFF
  * no RC packet for RC_TIMEOUT_S     -> OFF, never "hold the last value"
  * no link to the autopilot at all   -> OFF

A seeker that keeps running because it stopped hearing the switch is precisely
the failure this exists to prevent. Note the asymmetry that makes it safe: the
ON decision needs fresh positive evidence, while OFF is the default that needs
none.

THE SCHMITT TRIGGER IS NOT COSMETIC. A switch parked on the threshold, or a
noisy channel, would otherwise flip every frame -- and each flip of the record
switch closes one .rawrec episode and opens another, so a jittering channel
would shred a sortie into hundreds of files. Crossing by RC_HYSTERESIS_US is
required to flip; staying within it holds the current state.
"""
import pathlib
import sys

_REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))
import config


class RcArm:
    """One switch. Call update() once per frame; read .armed any time."""

    def __init__(self, chan, threshold_us, invert=False, timeout_s=None,
                 hysteresis_us=None, name="switch"):
        self.chan = int(chan)
        self.threshold = float(threshold_us)
        self.invert = bool(invert)
        self.timeout_s = float(config.RC_TIMEOUT_S if timeout_s is None else timeout_s)
        self.hyst = float(config.RC_HYSTERESIS_US
                          if hysteresis_us is None else hysteresis_us)
        self.name = name
        self.armed = False
        self.last_us = None
        self.transitions = 0
        self.no_rc_frames = 0

    def update(self, link):
        """-> (armed, reason). reason is for logging and the CSV, not control flow."""
        if link is None:
            return self._force_off("no_link")
        us, age = link.rc_us(self.chan)
        if us is None:
            return self._force_off("no_rc")
        if age > self.timeout_s:
            return self._force_off("rc_stale")

        self.last_us = us
        # Cross the threshold by hyst to flip; within it, hold. Which side counts
        # as "crossing" depends on the current state -- that is what makes it a
        # Schmitt trigger rather than a threshold with a dead zone.
        hi, lo = self.threshold + self.hyst, self.threshold - self.hyst
        above = us >= hi if not self.armed else us > lo
        want = (not above) if self.invert else above
        if want != self.armed:
            self.armed = want
            self.transitions += 1
        return self.armed, ("ARMED" if self.armed else "off")

    def _force_off(self, reason):
        self.no_rc_frames += 1
        was, self.armed = self.armed, False
        if was:
            self.transitions += 1
        return False, reason

    def describe(self):
        return "ch%d %s %.0fus%s" % (self.chan, "<" if self.invert else ">",
                                     self.threshold,
                                     " (inverted)" if self.invert else "")


def make_arm_switches():
    """The rig's two switches, from config.py. Returns (algo, record)."""
    return (RcArm(config.ARM_RC_CHANNEL, config.ARM_RC_US,
                  invert=config.RC_INVERT_ARM, name="algo"),
            RcArm(config.REC_RC_CHANNEL, config.REC_RC_US,
                  invert=config.RC_INVERT_REC, name="record"))
