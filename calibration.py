import os
import time
import json
import curses
import math
from controller import RobotController
from robotKinematics import RobotKinematics

CAL_FILE = os.path.join(os.path.dirname(__file__), "calibration.json")

REFERENCE_POSE = {"theta": 0.0, "phi": 0.0, "h": 8.26}

STEP_SUPER_COARSE = 5.0
STEP_COARSE = 1.0
STEP_FINE = 0.1
step = STEP_FINE

KEY_MAPPING = {
    ord('q'): ("s1", -1),
    ord('a'): ("s1", 1),
    ord('w'): ("s2", -1),
    ord('s'): ("s2", 1),
    ord('e'): ("s3", -1),
    ord('d'): ("s3", 1)
}

STEP_KEYS = {ord('c'): STEP_SUPER_COARSE, ord('z'): STEP_COARSE, ord('x'): STEP_FINE}
PRINT_KEY = ord('p')

def safe_exit(rc, stdscr):
    stdscr.addstr(9, 0, "Moving to safe pose before shutdown...      ")
    stdscr.refresh()

    theta = REFERENCE_POSE["theta"]
    phi = REFERENCE_POSE["phi"]
    h = REFERENCE_POSE["h"]

    rc.robot.solve_inverse_kinematics_spherical(theta, phi, h)
    angles = [
        math.degrees(math.pi * 0.5 - rc.robot.theta1),
        math.degrees(math.pi * 0.5 - rc.robot.theta2),
        math.degrees(math.pi * 0.5 - rc.robot.theta3),
    ]
    rc.set_motor_angles(*angles)
    time.sleep(0.5)

    rc.disarm()

    with open(CAL_FILE, "w") as f:
        json.dump(rc.cal, f, indent=2)

    stdscr.addstr(10, 0, f"Calibration saved to {CAL_FILE}                  ")
    stdscr.refresh()
    time.sleep(1)

def main(stdscr):
    global step

    curses.cbreak()
    stdscr.nodelay(True)
    stdscr.clear()
    stdscr.addstr(0, 0, "=== REAL-TIME CALIBRATION ===")
    stdscr.addstr(1, 0, "q/a: servo1  w/s: servo2  e/d: servo3")
    stdscr.addstr(2, 0, "c: super coarse  z: coarse  x: fine  p: print offsets")
    stdscr.addstr(3, 0, "Ctrl+C to exit\n")
    stdscr.refresh()

    model = RobotKinematics()
    rc = RobotController(model, calibration_file=CAL_FILE)
    rc.arm()

    try:
        while True:
            theta = REFERENCE_POSE["theta"]
            phi = REFERENCE_POSE["phi"]
            h = REFERENCE_POSE["h"]

            rc.robot.solve_inverse_kinematics_spherical(theta, phi, h)
            angles = [
                math.degrees(math.pi * 0.5 - rc.robot.theta1),
                math.degrees(math.pi * 0.5 - rc.robot.theta2),
                math.degrees(math.pi * 0.5 - rc.robot.theta3),
            ]
            rc.set_motor_angles(*angles)

            key = stdscr.getch()
            if key != -1:
                if key in KEY_MAPPING:
                    servo, direction = KEY_MAPPING[key]
                    rc.cal[servo]["offset"] += direction * step
                    stdscr.addstr(5, 0, f"Updated {servo} offset: {rc.cal[servo]['offset']:.2f}      ")

                if key in STEP_KEYS:
                    step = STEP_KEYS[key]
                    stdscr.addstr(6, 0, f"Step size set to {step}      ")

                if key == PRINT_KEY:
                    stdscr.addstr(
                        7, 0,
                        f"Offsets: s1={rc.cal['s1']['offset']:.2f} "
                        f"s2={rc.cal['s2']['offset']:.2f} "
                        f"s3={rc.cal['s3']['offset']:.2f}        "
                    )

                stdscr.refresh()

            time.sleep(0.05)

    except KeyboardInterrupt:
        stdscr.addstr(8, 0, "Ctrl+C detected, exiting safely...")
        stdscr.refresh()
        safe_exit(rc, stdscr)

if __name__ == "__main__":
    curses.wrapper(main)
