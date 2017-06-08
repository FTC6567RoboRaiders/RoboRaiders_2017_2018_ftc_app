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
    }

    public void cosineEncodersForward(int distance, double power) {

        int DIAMETER = 4;
        int GEAR_RATIO = 1;
        int PULSES = 1120;
        double CIRCUMFERENCE = Math.PI * DIAMETER;
        double ROTATIONS = (distance / CIRCUMFERENCE) * GEAR_RATIO;
        double COUNTS = PULSES * ROTATIONS;

        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        setMotorPower(power, power);

        while (motorLeft.getCurrentPosition() < COUNTS) {

            int i = motorLeft.getCurrentPosition();
            setMotorPower(((-Math.cos(i * ((2 * Math.PI) / COUNTS)) * power)) + power,
                    ((-Math.cos(i * ((2 * Math.PI) / COUNTS))) * power) + power);
        }

        setMotorPower(0, 0);
    }

    public void cosineEncodersBackward(int distance, double power) {

        int DIAMETER = 4;
        int GEAR_RATIO = 1;
        int PULSES = 1120;
        double CIRCUMFERENCE = Math.PI * DIAMETER;
        double ROTATIONS = (distance / CIRCUMFERENCE) * GEAR_RATIO;
        double COUNTS = PULSES * ROTATIONS;

        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        setMotorPower(-power, -power);

        while (motorLeft.getCurrentPosition() > COUNTS) {

            int i = motorLeft.getCurrentPosition();
            setMotorPower(-((-Math.cos(-i * ((2 * Math.PI) / COUNTS)) * power) + power),
                    -((-Math.cos(-i * ((2 * Math.PI) / COUNTS)) * power) + power));
        }

        setMotorPower(0, 0);
    }

    public void setMotorPower(double left, double right) {

        motorLeft.setPower(left);
        motorRight.setPower(right);
    }
}