package org.firstinspires.ftc.teamcode; // This line imports necessary software for this op mode.

/**
 * Created by Robotics Club on 11/30/2015.
 */



// IMPORTANT NOTE: IN ALL OF THE AUTONOMOUS OP MODES, THE FRONT OF THE ROBOT IS REALLY THE BACK. THAT IS WHY WHEN THE
// POWER IS A NEGATIVE VALUE THE ROBOT IS MOVING "FORWARD".



// This is a sample autonomous op mode.

public class TrueProphetClimberBlue extends TrueProphetAutonomousHeader { // This line establishes the name of the op mode and extends
    // the header file "TrueProphetOp", which in turn extends the
    // header file "LinearOpMode", in order to access all of the
    // information and public voids in "TrueProphetOp" and to
    // create an autonomous op mode.

    @Override
    public void runOpMode() throws InterruptedException { // This section of the code has both the
        // initialization routine the robot undergoes
        // and the main autonomous program that runs
        // in a linear fashion.

        teamColor = 1; // Because this op mode is to be used when we are on the blue alliance, teamColor is
        // 1, or blue.

        initEverything(); // This implementation of the public void initEverything initializes everything.

        while (sensorGyro.isCalibrating()) { // "While the gyro sensor is calibrating in the initialization
            // routine...

            Thread.sleep(50); // ...the program waits 50 milliseconds for it to calibrate...

            telemetry.addData("Calibrated", false); // ...the fact that calibration is false (the gyro
            // sensor is not done calibrating yet) is returned
            // to the driver controller phone...
        }

        telemetry.addData("Calibrated", true); // ...once the program has exited the while loop (the gyro
        // sensor is done calibrating) the fact that calibration
        // is true is returned to the driver controller phone so
        // that the coach can start the main program."

        waitForStart(); // Everything before this line is the initialization routine the robot undergoes,
        // while everything after it is the main autonomous program.

        Thread.sleep(5000); // "The program delays for 5 seconds for the robot to avoid colliding with other
        // robots...

        moveWheels(35, -1.0);     // ...the robot then moves forward 35 centimeters at a speed of 1.0...
        Thread.sleep(200);        // ...the program pauses for 200 milliseconds...
        setDriveTrainPower(0, 0); // ...the robot stops...
        Thread.sleep(200);        // ...and pauses for 200 milliseconds...

        gyroTurnRight(38, 1.0);   // ...the robot then turns right 38 degrees at a speed of 1.0...
        Thread.sleep(200);        // ...the program pauses for 200 milliseconds...
        setDriveTrainPower(0, 0); // ...the robot stops...
        Thread.sleep(200);        // ...and pauses for 200 milliseconds...

        gyroTurnLeft(1, 1.0);     // ...the robot then turns left 1 degree at a speed of 1.0 (this
        // 'un-twists' the treads)...
        Thread.sleep(200);        // ...the program pauses for 200 milliseconds...
        setDriveTrainPower(0, 0); // ...the robot stops...
        Thread.sleep(200);        // ...and pauses for 200 milliseconds...

        moveWheels(231, -1.0);    // ...the robot then moves forward 231 centimeters at a speed of 1.0...
        Thread.sleep(200);        // ...the program pauses for 200 milliseconds...
        setDriveTrainPower(0, 0); // ...the robot stops...
        Thread.sleep(200);        // ...and pauses for 200 milliseconds...

        gyroTurnRight(41, 1.0);   // ...the robot then turns right 41 degrees at a speed of 1.0...
        Thread.sleep(200);        // ...the program pauses for 200 milliseconds...
        setDriveTrainPower(0, 0); // ...the robot stops...
        Thread.sleep(200);        // ...and pauses for 200 milliseconds...

        gyroTurnLeft(1, 1.0);     // ...the robot then turns left 1 degree at a speed of 1.0 (this
        // 'un-twists' the treads)...
        Thread.sleep(200);        // ...the program pauses for 200 milliseconds...
        setDriveTrainPower(0, 0); // ...the robot stops...
        Thread.sleep(200);        // ...and pauses for 200 milliseconds...

        moveWheels(46, -1.0);     // ...the robot then moves forward 46 centimeters at a speed of 1.0...
        Thread.sleep(200);        // ...the program pauses for 200 milliseconds...
        setDriveTrainPower(0, 0); // ...the robot stops...
        Thread.sleep(200);        // ...and pauses for 200 milliseconds...

        servoClimberRight.setPosition(1.0); // ...servoClimberRight is then set to its 'down' position so as
        // not to interfere with servoClimberFront...
        Thread.sleep(200); // ...the program pauses for 200 milliseconds to allow for this...

        servoClimberFront.setPosition(0.0); // ...servoClimberFront is then set to its 'up' position is release
        // the climbers into the shelter...
        Thread.sleep(4000); // ...the program pauses for 4 seconds to allow for the release of the climbers...

        servoClimberFront.setPosition(1.0); // ...servoClimberFront is then reset to its 'down' position...
        Thread.sleep(200); // ...the program pauses for 200 milliseconds to allow for this, then ends."

    }
}