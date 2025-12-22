import os
import json
import time
import math
from adafruit_servokit import ServoKit

def load_calibration(path: str):
    if not os.path.exists(path):
        # Safe defaults (symmetrical, conservative)
        return {
            "s1": {"neutral": 54.0, "gain": 1.0, "offset": 0.0, "min": 10.0, "max": 100.0},
            "s2": {"neutral": 54.0, "gain": 1.0, "offset": 0.0, "min": 10.0, "max": 100.0},
            "s3": {"neutral": 54.0, "gain": 1.0, "offset": 0.0, "min": 10.0, "max": 100.0},
        }
    with open(path, "r") as f:
        return json.load(f)

def apply_cal(angle_cmd: float, cal: dict) -> float:
    """
    angle_cmd: angle from kinematics (model space)
    cal: per-servo calibration dict
    """
    neutral = float(cal.get("neutral", 54.0))
    gain = float(cal.get("gain", 1.0))
    offset = float(cal.get("offset", 0.0))
    amin = float(cal.get("min", 0.0))
    amax = float(cal.get("max", 180.0))

    # Neutral-centered scale + offset
    a = neutral + gain * (angle_cmd - neutral) + offset

    # Safety clamp in servo-command space
    if a < amin: a = amin
    if a > amax: a = amax
    return a

class RobotController:
    def __init__(self, model, calibration_file=None):
        self.robot = model
        self.kit = ServoKit(channels=16)

        # Assign servos
        self.s1 = self.kit.servo[13]
        self.s2 = self.kit.servo[15]
        self.s3 = self.kit.servo[14]

        for s in (self.s1, self.s2, self.s3):
            s.actuation_range = 270
            s.set_pulse_width_range(500, 2500)

        if calibration_file is None:
            calibration_file = os.path.join(os.path.dirname(__file__), "calibration.json")

        self.calibration_file = calibration_file
        self.cal = load_calibration(calibration_file)

        self.armed = False
        print(f"RobotController ready. Calibration loaded from {self.calibration_file}")

    # -----------------------------
    # Servo lifecycle
    # -----------------------------
    def arm(self, neutral_hold_s: float = 0.5):
        if self.armed:
            return
        self.armed = True

        # Neutral-first: prevents startup twitch / reversal
        n1 = float(self.cal["s1"].get("neutral", 54.0))
        n2 = float(self.cal["s2"].get("neutral", 54.0))
        n3 = float(self.cal["s3"].get("neutral", 54.0))
        self._force_angles(n1, n2, n3)
        time.sleep(neutral_hold_s)

        print("Servos armed (neutral-first)")

    def disarm(self, go_neutral: bool = True):
        if not self.armed:
            return
        print("Disarming servos")
        try:
            if go_neutral:
                n1 = float(self.cal["s1"].get("neutral", 54.0))
                n2 = float(self.cal["s2"].get("neutral", 54.0))
                n3 = float(self.cal["s3"].get("neutral", 54.0))
                self._force_angles(n1, n2, n3)
                time.sleep(0.25)

            for s in (self.s1, self.s2, self.s3):
                s.angle = None
        except Exception as e:
            print("Disarm error:", e)
        self.armed = False

    # -----------------------------
    # Low-level angle setter
    # -----------------------------
    def _force_angles(self, a1, a2, a3):
        # Apply calibration and send to servos
        self.s1.angle = apply_cal(a1, self.cal["s1"])
        self.s2.angle = apply_cal(a2, self.cal["s2"])
        self.s3.angle = apply_cal(a3, self.cal["s3"])

    def set_motor_angles(self, a1, a2, a3):
        if not self.armed:
            return
        self._force_angles(a1, a2, a3)

    # -----------------------------
    # Motion commands
    # -----------------------------
    def Goto_N_time_spherical(self, theta, phi, h):
        if not self.armed:
            return
        self.robot.solve_inverse_kinematics_spherical(theta, phi, h)
        angles = [
            math.degrees(math.pi * 0.5 - self.robot.theta1),
            math.degrees(math.pi * 0.5 - self.robot.theta2),
            math.degrees(math.pi * 0.5 - self.robot.theta3),
        ]
        self.set_motor_angles(*angles)

    def Goto_time_spherical(self, theta, phi, h, t=0.5):
        if not self.armed:
            return
        self.robot.solve_inverse_kinematics_spherical(theta, phi, h)
        angles = [
            math.degrees(math.pi * 0.5 - self.robot.theta1),
            math.degrees(math.pi * 0.5 - self.robot.theta2),
            math.degrees(math.pi * 0.5 - self.robot.theta3),
        ]
        self.interpolate_time(angles, duration=t)

    def interpolate_time(self, target_angles, duration=0.3):
        if not self.armed:
            return
        current = [self.s1.angle, self.s2.angle, self.s3.angle]
        steps = max(1, int(duration / 0.01))
        for i in range(steps + 1):
            u = i / steps
            angles = [c + (ta - c) * u for c, ta in zip(current, target_angles)]
            self._force_angles(*angles)
            time.sleep(duration / steps)
