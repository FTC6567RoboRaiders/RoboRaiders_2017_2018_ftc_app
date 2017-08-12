package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import java.util.Locale;

/**
 * Created by Katelin Zichittella on 6/6/2017.
 */

public abstract class AutonomousHeaderNew extends LinearOpMode {

    DcMotor motorLeft, motorRight;
    BNO055IMU imu;

    public void initialize() {

        motorLeft = hardwareMap.dcMotor.get("motorLeft");
        motorRight = hardwareMap.dcMotor.get("motorRight");
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);

        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorRight.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Initialized", true);
        telemetry.update();
    }

    public void cosineEncodersForward(int distance, double maxPower) {

        if (opModeIsActive()) {

            int DIAMETER = 4;
            int GEAR_RATIO = 1;
            int PULSES = 1120;
            double CIRCUMFERENCE = Math.PI * DIAMETER;
            double ROTATIONS = (distance / CIRCUMFERENCE) * GEAR_RATIO;
            double COUNTS = PULSES * ROTATIONS;

            motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            setMotorPower(maxPower, maxPower);

            while (motorRight.getCurrentPosition() < COUNTS && opModeIsActive()) {

                int currentEncoderCount = motorRight.getCurrentPosition();

                setMotorPower((-((maxPower - 0.1) / 2) * (Math.cos(currentEncoderCount * ((2 * Math.PI) / COUNTS)))) + ((maxPower + 0.1) / 2),
                        (-((maxPower - 0.1) / 2) * (Math.cos(currentEncoderCount * ((2 * Math.PI) / COUNTS)))) + ((maxPower + 0.1) / 2));

                telemetry.addData("In While Loop", true);
                telemetry.addData("COUNTS", COUNTS);
                telemetry.addData("Current Position", motorRight.getCurrentPosition());
                telemetry.addData("currentEncoderCount", currentEncoderCount);
                telemetry.addData("Left Speed", motorLeft.getPower());
                telemetry.addData("Right Speed", motorRight.getPower());
                telemetry.update();
            }

            setMotorPower(0, 0);
        }
    }

    public void halfCosineEncodersForward(int distance, double maxPower) {

        if (opModeIsActive()) {

            int DIAMETER = 4;
            int GEAR_RATIO = 1;
            int PULSES = 1120;
            double CIRCUMFERENCE = Math.PI * DIAMETER;
            double ROTATIONS = (distance / CIRCUMFERENCE) * GEAR_RATIO;
            double COUNTS = PULSES * ROTATIONS;

            motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            setMotorPower(maxPower, maxPower);

            while (motorRight.getCurrentPosition() < COUNTS && opModeIsActive()) {

                int currentEncoderCount = motorRight.getCurrentPosition();

                setMotorPower((-((maxPower - 0.1) / 2) * (Math.cos((currentEncoderCount + COUNTS) * ((2 * Math.PI) / (COUNTS * 2))))) + ((maxPower + 0.1) / 2),
                        (-((maxPower - 0.1) / 2) * (Math.cos((currentEncoderCount + COUNTS) * ((2 * Math.PI) / (COUNTS * 2))))) + ((maxPower + 0.1) / 2));

                telemetry.addData("In While Loop", true);
                telemetry.addData("COUNTS", COUNTS);
                telemetry.addData("Current Position", motorRight.getCurrentPosition());
                telemetry.addData("currentEncoderCount", currentEncoderCount);
                telemetry.addData("Left Speed", motorLeft.getPower());
                telemetry.addData("Right Speed", motorRight.getPower());
                telemetry.update();
            }

            setMotorPower(0, 0);
        }
    }

    public void cosineEncodersBackward(int distance, double maxPower) {

        if (opModeIsActive()) {

            int DIAMETER = 4;
            int GEAR_RATIO = 1;
            int PULSES = 1120;
            double CIRCUMFERENCE = Math.PI * DIAMETER;
            double ROTATIONS = (distance / CIRCUMFERENCE) * GEAR_RATIO;
            double COUNTS = PULSES * ROTATIONS;

            motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            setMotorPower(-maxPower, -maxPower);

            while (-motorRight.getCurrentPosition() < COUNTS && opModeIsActive()) {

                int currentEncoderCount = -motorRight.getCurrentPosition();

                setMotorPower(-((-((maxPower - 0.1) / 2) * (Math.cos(currentEncoderCount * ((2 * Math.PI) / COUNTS)))) + ((maxPower + 0.1) / 2)),
                        -((-((maxPower - 0.1) / 2) * (Math.cos(currentEncoderCount * ((2 * Math.PI) / COUNTS)))) + ((maxPower + 0.1) / 2)));

                telemetry.addData("In While Loop", true);
                telemetry.addData("COUNTS", COUNTS);
                telemetry.addData("Current Position", -motorRight.getCurrentPosition());
                telemetry.addData("currentEncoderCount", currentEncoderCount);
                telemetry.addData("Left Speed", motorLeft.getPower());
                telemetry.addData("Right Speed", motorRight.getPower());
                telemetry.update();
            }

            setMotorPower(0, 0);
        }
    }

    public void halfCosineEncodersBackward(int distance, double maxPower) {

        if (opModeIsActive()) {

            int DIAMETER = 4;
            int GEAR_RATIO = 1;
            int PULSES = 1120;
            double CIRCUMFERENCE = Math.PI * DIAMETER;
            double ROTATIONS = (distance / CIRCUMFERENCE) * GEAR_RATIO;
            double COUNTS = PULSES * ROTATIONS;

            motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            setMotorPower(-maxPower, -maxPower);

            while (-motorRight.getCurrentPosition() < COUNTS && opModeIsActive()) {

                int currentEncoderCount = -motorRight.getCurrentPosition();

                setMotorPower(-((-((maxPower - 0.1) / 2) * (Math.cos((currentEncoderCount + COUNTS) * ((2 * Math.PI) / (COUNTS * 2))))) + ((maxPower + 0.1) / 2)),
                        -((-((maxPower - 0.1) / 2) * (Math.cos((currentEncoderCount + COUNTS) * ((2 * Math.PI) / (COUNTS * 2))))) + ((maxPower + 0.1) / 2)));

                telemetry.addData("In While Loop", true);
                telemetry.addData("COUNTS", COUNTS);
                telemetry.addData("Current Position", -motorRight.getCurrentPosition());
                telemetry.addData("currentEncoderCount", currentEncoderCount);
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

    public void composeTelemetry() {

        telemetry.addAction(new Runnable() { @Override public void run()
        {
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = imu.getGravity();
        }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel*gravity.xAccel
                                        + gravity.yAccel*gravity.yAccel
                                        + gravity.zAccel*gravity.zAccel));
                    }
                });
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}