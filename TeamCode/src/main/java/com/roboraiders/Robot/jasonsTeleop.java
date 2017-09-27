package com.roboraiders.Robot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Jason Sember on 9/23/2017.
 */

@TeleOp

public class jasonsTeleop extends OpMode {
    Robot robot = new Robot();
    DcMotor motorBackLeft, motorBackRight, motorFrontLeft, motorFrontRight;

    @Override
    public void init() { /*This is the initialization routine that the robot undergoes. */

        motorBackLeft = hardwareMap.dcMotor.get("left_Back");           // These lines establish a link between
        motorBackRight = hardwareMap.dcMotor.get("right_Back");         // the code and the hardware for the
        motorFrontLeft = hardwareMap.dcMotor.get("left_Front");         // motors. The names in quotations are
        motorFrontRight = hardwareMap.dcMotor.get("right_Front");       //the names of the motors we set on the phone.

        motorBackRight.setDirection(DcMotor.Direction.REVERSE);             //These lines reverse the right motors
        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);            //in order to negate the fact that the
                                                                            //motors are placed on the robot
                                                                            //to mirror each other.


        telemetry.addData("We is good", "now we dank", "i can't believe it's not butter");
        telemetry.update() ;
    }

    @Override
    public void loop() {

        float motorLeftback = gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x;    // These lines establish the joystick input values as
        float motorRightback = gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x;   // the float variables "backLeft", "baclRight", "frontLeft", and "frontRight", which
        float motorLeftfront = gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x;   //correspond to the back left, back right, front left,
        float motorRightfront = gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x;  // and front right wheels of the robot.

        float maxpwrA = Math.max(Math.abs(motorLeftback),Math.abs(motorRightback));
        float maxpwrB = Math.max (Math.abs(motorLeftfront), Math.abs( motorRightfront));
        float maxpwr = Math.max(Math.abs (maxpwrA), Math.abs(maxpwrB));

            motorLeftback = motorLeftback / maxpwr;
            motorRightback = motorRightback / maxpwr;
            motorLeftfront = motorLeftfront / maxpwr;
            motorRightfront = motorRightfront / maxpwr;

        motorLeftback = (float) scaleInput(motorLeftback);// These lines scale the joystick input values in the
        motorRightback = (float) scaleInput(motorRightback);// resulting floats to the values in the array in the
        motorLeftfront = (float) scaleInput(motorLeftfront); // double below, which are the only ones the program
        motorRightfront = (float) scaleInput(motorRightfront);
        setMotorPower( motorLeftback, motorRightback, motorRightfront, motorLeftfront);  // This line is an implementation of the public void
        // "setMotorPower" below. It sets the power of the motors to the joystick input values in
        // the floats.
    }

    @Override
    public void stop() {

    }

    public void setMotorPower(float motorLeftback, float motorRightback, float motorLeftfront, float motorRightfront) { // This public void, when implemented
                                                                                                    // above, sets the power of the four motors.
                                                                                                    // Whatever is inputted into each of the four
                                                                                                    // parameters above is then substituted
                                                                                                    // into its corresponding spot in the
                                                                                                    // public void.

        motorBackLeft.setPower(motorLeftback);   // These lines set the power of each motor to the desired power.
        motorBackRight.setPower(motorRightback);
        motorFrontLeft.setPower(motorLeftfront);
        motorFrontRight.setPower(motorRightfront);
    }

    double scaleInput(double dVal) {        // When implemented above, this double scales the joystick input values
                                            // in the floats.

        double[] scaleArray = {0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00};

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);

        // index should be positive.
        if (index < 0) {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 16) {
            index = 16;
        }

        // get value from the array.
        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        // return scaled value.
        return dScale;
    }
}