# Info
This project runs on Arduino using PlatformIO. All servo motors and interrupt pins are mapped. Positions of the motors are not recorded and a 1 is left as a placeholder for now.
The code takes in a message from the UR robot when the top motor is ready and interprets it based on the size of the plate. Plates are stacked using a state machine built on "phase flags" which tell the arduino which phase of stacking we are in. There are 4 identical functions for each column of the stacker which advance the stacking process through its phases depending on the current position of the actuated motors. This state machine individualised for each column allows for simultaneous actuation of motors on separate columns instead of waiting for the process to finish on a column before proceeding. Plates will be stacked alternatively on the left and right columns. When the plate count on a column reaches 5, the plates will be carried to the bottom of the stacker and flushed out.

## Important note!
**The code has not been tested on the physical prototype, therefore bugs are certain to be present. Please keep in mind.**

# How to use
Clone repository. Use Visual Studio Code and install the PlatformIO plugin. Open the AgarTronDuino folder which contains the *platformio.ini* file and upload to board.