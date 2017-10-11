package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.roboraiders.Robot.Robot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by Nick on 10/8/17.
 */

@Autonomous

public class SensorTest extends LinearOpMode {

    public Robot robot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {

        robot.initialize(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {

            robot.currState = robot.digitalTouch.getState();

            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(robot.relicTemplate);
            robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            robot.colorSensor.red();
            robot.colorSensor.blue();

            if (vuMark.equals(RelicRecoveryVuMark.LEFT)) {

                robot.pictograph = "LEFT";
            }
            else if (vuMark.equals(RelicRecoveryVuMark.CENTER)) {

                robot.pictograph = "CENTER";
            }
            else if (vuMark.equals(RelicRecoveryVuMark.RIGHT)) {

                robot.pictograph = "RIGHT";
            }
            else {

                robot.pictograph = "UNKNOWN";
            }

            telemetry.addData("Red", robot.colorSensor.red());
            telemetry.addData("Blue", robot.colorSensor.blue());
            telemetry.addData("Distance", robot.distanceSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("Pictograph", robot.pictograph);
            telemetry.addData("IMU Angle", robot.angles.firstAngle);
            telemetry.update();

            encodersStrafeRight(20, 0.5);

            robot.setDriveMotorPower(0.2, -0.2, -0.2, 0.2);

            if (robot.currState && robot.currState != robot.prevState) {

                robot.walls++;
                robot.prevState = robot.currState;
            }
            else if (!robot.currState && robot.currState != robot.prevState) {

                robot.prevState = robot.currState;
            }

            if (robot.walls == 2) {

                robot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0);
            }
        }
    }

    public void encodersStrafeRight(int distance, double power) {

        if (opModeIsActive()) {

            robot.motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            int DIAMETER = 4;
            int GEAR_RATIO = 1;
            int PULSES = 1120;
            double CIRCUMFERENCE = Math.PI * DIAMETER;
            double ROTATIONS = (distance / CIRCUMFERENCE) * GEAR_RATIO;
            double COUNTS = PULSES * ROTATIONS;

            COUNTS = COUNTS + Math.abs(robot.motorBackRight.getCurrentPosition());

            robot.setDriveMotorPower(power, -power, -power, power);

            while (robot.motorBackRight.getCurrentPosition() < COUNTS && opModeIsActive()) {

            }

            robot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0);
        }
    }
}