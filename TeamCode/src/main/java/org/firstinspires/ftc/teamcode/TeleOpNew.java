package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.roboraiders.Robot.Robot;

/**
 * Created by Jason Sember on 9/23/2017.
 */

@TeleOp

public class TeleOpNew extends OpMode {

    public Robot robot = new Robot();

    @Override
    public void init() {

        robot.initialize(hardwareMap);

        telemetry.addData("Initialized", true);
        telemetry.update() ;
    }

    /*@Override
    public void loop() {

        float LeftBack = gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x;
        float RightBack = gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x;
        float LeftFront = gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x;
        float RightFront = gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x;

        float maxpwrA = Math.max(Math.abs(LeftBack), Math.abs(LeftBack));
        float maxpwrB = Math.max (Math.abs(LeftFront), Math.abs(RightFront));
        float maxpwr = Math.max(Math.abs(maxpwrA), Math.abs(maxpwrB));

        LeftBack = LeftBack / maxpwr;
        RightBack = RightBack / maxpwr;
        LeftFront = LeftFront / maxpwr;
        RightFront = RightFront / maxpwr;

        LeftBack = (float) scaleInput(LeftBack);
        RightBack = (float) scaleInput(RightBack);
        LeftFront = (float) scaleInput(LeftFront);
        RightFront = (float) scaleInput(RightFront);

        robot.setDriveMotorPower(LeftBack, RightBack, RightFront, LeftFront);
    }*/

    @Override
    public void loop() {

        float LeftBack = gamepad1.left_stick_y - gamepad1.left_stick_x;
        float RightBack = gamepad1.right_stick_y + gamepad1.left_stick_x;
        float LeftFront = gamepad1.left_stick_y + gamepad1.left_stick_x;
        float RightFront = gamepad1.right_stick_y - gamepad1.left_stick_x;

        LeftBack = (float) scaleInput(LeftBack);
        RightBack = (float) scaleInput(RightBack);
        LeftFront = (float) scaleInput(LeftFront);
        RightFront = (float) scaleInput(RightFront);

        robot.setDriveMotorPower(LeftBack, RightBack, RightFront, LeftFront);
    }

    @Override
    public void stop() {

    }

    double scaleInput(double dVal) {
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