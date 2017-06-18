package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Katelin Zichittella on 6/6/2017.
 */

public abstract class AutonomousHeaderNew extends LinearOpMode {

    DcMotor motorLeft, motorRight;

    public void initialize() {

        motorLeft = hardwareMap.dcMotor.get("motorLeft");
        motorRight = hardwareMap.dcMotor.get("motorRight");

        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorRight.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Initialized", true);
        telemetry.update();
    }

    // Lower speeds need higher threshold values, but are best

    public void cosineEncodersForward(int distance, double power, double threshold) {

        if (opModeIsActive()) {

            int DIAMETER = 4;
            int GEAR_RATIO = 1;
            int PULSES = 1120;
            double CIRCUMFERENCE = Math.PI * DIAMETER;
            double ROTATIONS = (distance / CIRCUMFERENCE) * GEAR_RATIO;
            double COUNTS = PULSES * ROTATIONS;

            motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            setMotorPower(power, power);

            while (motorRight.getCurrentPosition() < COUNTS && opModeIsActive()) {

                int i = motorRight.getCurrentPosition();

                setMotorPower((-(power / 2) * (Math.cos((i + threshold) * ((2 * Math.PI) / (COUNTS + (threshold * 2)))))) + (power / 2),
                        (-(power / 2) * (Math.cos((i + threshold) * ((2 * Math.PI) / (COUNTS + (threshold * 2)))))) + (power / 2));

                telemetry.addData("In While Loop", true);
                telemetry.addData("COUNTS", COUNTS);
                telemetry.addData("Current Position", motorRight.getCurrentPosition());
                telemetry.addData("i", i);
                telemetry.addData("Left Speed", motorLeft.getPower());
                telemetry.addData("Right Speed", motorRight.getPower());
                telemetry.update();
            }

            setMotorPower(0, 0);
        }
    }

    public void halfCosineEncodersForward(int distance, double power, double threshold) { // Needs a slightly higher threshold, but is best

        if (opModeIsActive()) {

            int DIAMETER = 4;
            int GEAR_RATIO = 1;
            int PULSES = 1120;
            double CIRCUMFERENCE = Math.PI * DIAMETER;
            double ROTATIONS = (distance / CIRCUMFERENCE) * GEAR_RATIO;
            double COUNTS = PULSES * ROTATIONS;

            motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            setMotorPower(power, power);

            while (motorRight.getCurrentPosition() < COUNTS && opModeIsActive()) {

                int i = motorRight.getCurrentPosition();

                setMotorPower((-(power / 2) * (Math.cos((i + (((COUNTS * 2) + threshold) / 2)) * ((2 * Math.PI) / ((COUNTS * 2) + (threshold * 2)))))) + (power / 2),
                        (-(power / 2) * (Math.cos((i + (((COUNTS * 2) + threshold) / 2)) * ((2 * Math.PI) / ((COUNTS * 2) + (threshold * 2)))))) + (power / 2));

                telemetry.addData("In While Loop", true);
                telemetry.addData("COUNTS", COUNTS);
                telemetry.addData("Current Position", motorRight.getCurrentPosition());
                telemetry.addData("i", i);
                telemetry.addData("Left Speed", motorLeft.getPower());
                telemetry.addData("Right Speed", motorRight.getPower());
                telemetry.update();
            }

            setMotorPower(0, 0);
        }
    }

    public void cosineEncodersBackward(int distance, double power, double threshold) {

        if (opModeIsActive()) {

            int DIAMETER = 4;
            int GEAR_RATIO = 1;
            int PULSES = 1120;
            double CIRCUMFERENCE = Math.PI * DIAMETER;
            double ROTATIONS = (distance / CIRCUMFERENCE) * GEAR_RATIO;
            double COUNTS = PULSES * ROTATIONS;

            motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            setMotorPower(-power, -power);

            while (-motorRight.getCurrentPosition() < COUNTS && opModeIsActive()) {

                int i = -motorRight.getCurrentPosition();

                setMotorPower(-((-(power / 2) * (Math.cos((i + threshold) * ((2 * Math.PI) / (COUNTS + (threshold * 2)))))) + (power / 2)),
                        -((-(power / 2) * (Math.cos((i + threshold) * ((2 * Math.PI) / (COUNTS + (threshold * 2)))))) + (power / 2)));

                telemetry.addData("In While Loop", true);
                telemetry.addData("COUNTS", COUNTS);
                telemetry.addData("Current Position", -motorRight.getCurrentPosition());
                telemetry.addData("i", i);
                telemetry.addData("Left Speed", motorLeft.getPower());
                telemetry.addData("Right Speed", motorRight.getPower());
                telemetry.update();
            }

            setMotorPower(0, 0);
        }
    }

    public void halfCosineEncodersBackward(int distance, double power, double threshold) { // Needs a slightly higher threshold, but is best

        if (opModeIsActive()) {

            int DIAMETER = 4;
            int GEAR_RATIO = 1;
            int PULSES = 1120;
            double CIRCUMFERENCE = Math.PI * DIAMETER;
            double ROTATIONS = (distance / CIRCUMFERENCE) * GEAR_RATIO;
            double COUNTS = PULSES * ROTATIONS;

            motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            setMotorPower(-power, -power);

            while (-motorRight.getCurrentPosition() < COUNTS && opModeIsActive()) {

                int i = -motorRight.getCurrentPosition();

                setMotorPower(-((-(power / 2) * (Math.cos((i + (((COUNTS * 2) + threshold) / 2)) * ((2 * Math.PI) / ((COUNTS * 2) + (threshold * 2)))))) + (power / 2)),
                        -((-(power / 2) * (Math.cos((i + (((COUNTS * 2) + threshold) / 2)) * ((2 * Math.PI) / ((COUNTS * 2) + (threshold * 2)))))) + (power / 2)));

                telemetry.addData("In While Loop", true);
                telemetry.addData("COUNTS", COUNTS);
                telemetry.addData("Current Position", -motorRight.getCurrentPosition());
                telemetry.addData("i", i);
                telemetry.addData("Left Speed", motorLeft.getPower());
                telemetry.addData("Right Speed", motorRight.getPower());
                telemetry.update();
            }

            setMotorPower(0, 0);
        }
    }

    public void setMotorPower(double left, double right) {

        motorLeft.setPower(left);
        motorRight.setPower(right);
    }
}
/*for (int i = 500; i < COUNTS; i++) {

                motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                while (motorRight.getCurrentPosition() < 1) {

                    setMotorPower((-0.5 * (Math.cos(i * ((2 * Math.PI) / COUNTS) * power))) + (power / 2),
                            (-0.5 * (Math.cos(i * ((2 * Math.PI) / COUNTS) * power))) + (power / 2));

                    telemetry.addData("In Loop", true);
                    telemetry.addData("COUNTS", COUNTS);
                    telemetry.addData("Current Position", motorRight.getCurrentPosition());
                    telemetry.addData("i", i);
                    telemetry.addData("Left Speed", motorLeft.getPower());
                    telemetry.addData("Right Speed", motorRight.getPower());
                    telemetry.update();
                }
            }*/