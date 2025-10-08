# VoxControl Implementation Using the CrowBot-BOLT Robot 

[Quick Start](https://grovety.com/CrowBot/)

[File of the program](https://github.com/Grovety/CrowBot_GRC_program/blob/main/Bolt_grc_program.ino)

[How to flash the program](https://github.com/Grovety/CrowBot_GRC_program/blob/main/how_to_install_the_program.md)

[Comands List](https://grovety.com/CrowBot/)

## How it works

Once you flash our program to the CrowBot  (see [how to do it](https://github.com/Grovety/CrowBot_GRC_program/blob/main/how_to_install_the_program.md)), the robot can receive commands from the DevBoard via Bluetooth.  

1. Turn on the robot and the DevBoard.  
2. Enable Bluetooth on the robot by sliding the switch.  
3. The robot will automatically connect to the DevBoard, and a connection icon will appear on the board.  

![RC2](https://github.com/user-attachments/assets/907467d8-6e76-4b28-b7d1-531149297eba)

After the connection is established, the robot can process voice commands, which are then converted into movement commands or other actions.

You can use two modes to control CrowBot-BOLT:
1. Recognizing voice commands from a predefined list. 
2. Reading the board's tilt along the X and Y axes.

![RCax](https://github.com/user-attachments/assets/df23ef1b-fbf7-4044-8efd-d63a35861690)

## Voice Control

Demo: https://youtu.be/RoclNdVzP7A

The board "listens" to user commands through its built-in microphones and recognizes them using a neural network. The recognized command is transmitted to CrowBOT.

### Commands
| Voice Command     | Action |
|------------------|----------------------------------------------------------------------------------------------------------------------------------|
| **ROBOT WAKE UP**   | The DevBoard wakes up. The robot performs no action. |  
| **ROBOT SLEEP**     | The DevBoard goes into sleep mode and stops responding to voice commands. The robot performs no action. |  
| **MANUAL CONTROL**  | The DevBoard switches to tilt control mode. The robot enters tilt control mode and stops responding to voice movement commands. |  
| **VOICE CONTROL**   | Tilt control mode is disabled, returning to voice control mode. |  
| **GO RIGHT**       | Turn right. |  
| **GO LEFT**        | Turn left. |  
| **GO FORWARD**     | Move forward and then stop. |  
| **GO BACK**        | Move backward and then stop. |  
| **GO HOME**        | No action. |  
| **SLOWER SPEED**   | No action. |  
| **FASTER SPEED**   | No action. |  
| **LIGHTS ON**      | Turn on the LED backlight on the Ultrasonic Sensor (simulating headlights). |  
| **LIGHTS OFF**     | Turn off the LED backlight on the Ultrasonic Sensor (simulating headlights). |  
| **PLAY MUSIC**     | The robot plays a preset melody. |  

## Tilt mode

Demo: https://youtu.be/FIsmTgkFIoI

To activate tilt control mode, say 'Manual Control' or press the USER button. 

Once activated, the display on the DevBoard will update, and the robot will no longer respond to voice commands for movement.

> Make sure to place the DevBoard on a flat surface before starting, as it will calibrate when entering tilt control mode. When you see a circle on two intersecting lines appear on the screen, the calibration is done, and you’re ready to start controlling the robot.

Tilt the DevBoard forward, backward, left, or right to control the robot's movement. The tilt angle determines the robot's speed.

You can use voice commands 'Play Music' and 'Lights On' or 'Lights Off' in Tilt mode.

To quit say 'Voice Control' or press USER button.









