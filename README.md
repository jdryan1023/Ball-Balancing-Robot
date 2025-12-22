# Ball-Balancing-Robot
An educational Ball-Balancing Robot. This project introduces some of the core concepts of robotics: programming, inverse kinematics, computer vision, and PID control.

Discord: https://discord.com/invite/WJuUWsy6DJ

# Video Overview
<div align="center">
  <a href="https://www.youtube.com/watch?v=l92hJUUjWb0&t=6s">
    <img src="https://img.youtube.com/vi/l92hJUUjWb0/0.jpg" alt="Watch the video" width="640">
  </a>
</div>

# Pictures

<div align="center">
  <img src="https://github.com/user-attachments/assets/a4252720-81c4-4c18-85fc-d38fa5c8ed7a" alt="2" width="700" />
</div>
<div align="center">
  <img src="https://github.com/user-attachments/assets/8eb9a714-8835-46d8-bd33-cb98a85422c6" alt="1" width="700" />
</div>


# Build Instruction

The 3D models and print profile for a Bambu A1 printer can be found here: https://makerworld.com/en/models/1197770-ball-balancing-robot#profileId-1210633

The materials needed are:
| Part Name                     | Quantity |
|-------------------------------|----------|
| M2 x 8 Cap Head Socket Screw  | 6        |
| M2.5 x 10 Cap Head Socket Screw  | 4        |
| M3 x 5 Socket Head Screw      | 18       |
| M3 x 10 Socket Head Screw     | 3        |
| M3 x 15 Socket Head Screw     | 1        |
| M3 x 20 Socket Head Screw     | 3        |
| M4 x 20 Socket Head Screw     | 6        |
| M4 x 30 Socket Head Screw     | 6        |
| M5 x 30 Socket Head Screw     | 3        |
| M3 Hex Nut                   | 3        |
| M4 Nylock Nut                | 6        |
| M4 Hex Nut                   | 12       |
| M3 x 10 Standoff             | 9        |
| M3 x 15 Standoff             | 6        |
| M3 x 20 Standoff             | 3        |
| Standoff                     | 1        |
| 4-10 Bearing                 | 6        |
| Rubber Foot 12x9x9           | 3        |
| M5 Washer                    | 6        |

---
### Start-up 
1. Image Raspberry Pi with Raspberry Pi OS
2. Open CMD and run "Initial CMD Script".  This will pull all the necessary installations and make a .venv for the robot code to operate in.
3. GUI can be run from "Windows_Run_Gui_Silent"

### Motor Initialization and Calibration
1. With the motors on the bench and connected to the PCA per the wiring diagram, run "zeromotor.py".  This will put the motors in a neutral position.
2. Power down the PCA and do not rotate the motors. Install motors onto printed platform. Install the RC horns at the same angle (best case).
3. Run calibration.py. This will put the platform in a "zero" pose. We need to level the platform applying motor offsets, this can be achieved by using the key mapping below to jog the individual motors. 
	NOTE: EXIT ROUTINE WITH CRTL + C!!!!!!!!!!

	STEP_KEYS = {ord('c'): STEP_SUPER_COARSE, ord('z'): STEP_COARSE, ord('x'): STEP_FINE}
	STEP_SUPER_COARSE = 5.0 
	STEP_COARSE = 1.0
	STEP_FINE = 0.1

	KEY_MAPPING for motor jog = 
    		ord('q'): ("s1", -1),
    		ord('a'): ("s1", 1),
    		ord('w'): ("s2", -1),
    		ord('s'): ("s2", 1),
    		ord('e'): ("s3", -1),
    		ord('d'): ("s3", 1)
4. This will save the motor calibration offsets to a calibration.JSON and apply to the controller.py