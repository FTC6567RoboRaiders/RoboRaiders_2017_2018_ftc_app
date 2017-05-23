package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Katelin Zichittella on 11/20/2016.
 */

public abstract class TalosAutonomousHeader extends LinearOpMode {

    DcMotor motorBackLeft, motorBackRight, motorFrontLeft, motorFrontRight,
            motorShooter, motorSweeper, motorLift1, motorLift2;
    Servo servoBeacon, servoGate, servoLiftLeft, servoLiftRight;
    GyroSensor sensorGyro;

    byte[] rangeSensorLeftCache;
    byte[] rangeSensorRightCache;

    I2cDevice rangeSensorLeft;
    I2cDevice rangeSensorRight;
    I2cDeviceSynch rangeSensorLeftReader;
    I2cDeviceSynch rangeSensorRightReader;

    byte[] colorSensorLeftCache;
    byte[] colorSensorRightCache;
    byte[] colorSensorFrontCache;

    I2cDevice colorSensorLeft;
    I2cDevice colorSensorRight;
    I2cDevice colorSensorFront;
    I2cDeviceSynch colorSensorLeftReader;
    I2cDeviceSynch colorSensorRightReader;
    I2cDeviceSynch colorSensorFrontReader;

    public void initialize() {

        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorShooter = hardwareMap.dcMotor.get("motorShooter");
        motorSweeper = hardwareMap.dcMotor.get("motorSweeper");
        motorLift1 = hardwareMap.dcMotor.get("motorLift1");
        motorLift2 = hardwareMap.dcMotor.get("motorLift2");
        rangeSensorLeft = hardwareMap.i2cDevice.get("rangeSensorLeft");
        rangeSensorRight = hardwareMap.i2cDevice.get("rangeSensorRight");
        colorSensorLeft = hardwareMap.i2cDevice.get("colorSensorLeft");
        colorSensorRight = hardwareMap.i2cDevice.get("colorSensorRight");
        colorSensorFront = hardwareMap.i2cDevice.get("colorSensorFront");
        sensorGyro = hardwareMap.gyroSensor.get("sensorGyro");
        servoBeacon = hardwareMap.servo.get("servoBeacon");
        servoGate = hardwareMap.servo.get("servoGate");
        servoLiftLeft = hardwareMap.servo.get("servoLiftLeft");
        servoLiftRight = hardwareMap.servo.get("servoLiftRight");

        colorSensorLeftReader = new I2cDeviceSynchImpl(colorSensorLeft, I2cAddr.create8bit(0x3c), false);
        colorSensorRightReader = new I2cDeviceSynchImpl(colorSensorRight, I2cAddr.create8bit(0x3e), false);
        colorSensorFrontReader = new I2cDeviceSynchImpl(colorSensorFront, I2cAddr.create8bit(0x42), false);
        rangeSensorLeftReader = new I2cDeviceSynchImpl(rangeSensorLeft, I2cAddr.create8bit(0x28), false);
        rangeSensorRightReader = new I2cDeviceSynchImpl(rangeSensorRight, I2cAddr.create8bit(0x30), false);

        colorSensorLeftReader.write8(3, 0);
        colorSensorRightReader.write8(3, 0);
        colorSensorFrontReader.write8(3, 1);

        colorSensorLeftReader.engage();
        colorSensorRightReader.engage();
        colorSensorFrontReader.engage();
        rangeSensorLeftReader.engage();
        rangeSensorRightReader.engage();

        motorBackRight.setDirection(DcMotor.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        motorShooter.setDirection(DcMotor.Direction.REVERSE);
        servoBeacon.setPosition(0.0);
        servoGate.setPosition(0.0);
        servoLiftLeft.setPosition(0.70);
        servoLiftRight.setPosition(0.67);
    }

    public void calibrateGyro () throws InterruptedException {

        sensorGyro.calibrate();

        while (sensorGyro.isCalibrating()) {

            Thread.sleep(50);

            telemetry.addData("Calibrated", false);
            telemetry.update();
        }

        telemetry.addData("Calibrated", true);
        telemetry.update();
    }

    public void lineFollowerTwoSensors (double distance) {

        if (opModeIsActive()) {

            setMotorPower(0.24, 0.24);

            rangeSensorLeftCache = rangeSensorLeftReader.read(0x04, 1);
            rangeSensorRightCache = rangeSensorRightReader.read(0x04, 1);

            while (((rangeSensorLeftCache[0] & 0xFF) > distance || (rangeSensorRightCache[0] & 0xFF) > distance) && opModeIsActive()) {

                rangeSensorLeftCache = rangeSensorLeftReader.read(0x04, 1);
                rangeSensorRightCache = rangeSensorRightReader.read(0x04, 1);

                /*telemetry.addData("LeftRange", rangeSensorLeftCache[0] & 0xFF);
                telemetry.addData("RightRange", rangeSensorRightCache[0] & 0xFF);
                telemetry.update();*/

                colorSensorLeftCache = colorSensorLeftReader.read(0x08, 1);
                colorSensorRightCache = colorSensorRightReader.read(0x08, 1);

                /*telemetry.addData("LeftColor", colorSensorLeftCache[0] & 0xFF);
                telemetry.addData("RightColor", colorSensorRightCache[0] & 0xFF);
                telemetry.update();*/

                if ((colorSensorLeftCache[0] & 0xFF) < 45 && (colorSensorRightCache[0] & 0xFF) < 45) { // black

                    setMotorPower(0.24, 0.24);
                }

                else if ((colorSensorLeftCache[0] & 0xFF) >= 45) { // white

                    setMotorPower(0, 0.2);
                }

                else if ((colorSensorRightCache[0] & 0xFF) >= 45) { // white

                    setMotorPower(0.2, 0);
                }

                else {

                    setMotorPower(0.24, 0.24);
                }
            }

            setMotorPower(0.0, 0.0);
        }
    }

    public void moveUntilWhiteLineStraight (double power) {

        if (opModeIsActive()) {

            setMotorPower(power, power);

            colorSensorLeftCache = colorSensorLeftReader.read(0x08, 1);
            colorSensorRightCache = colorSensorRightReader.read(0x08, 1);

            while (((colorSensorLeftCache[0] & 0xFF) < 45 && (colorSensorRightCache[0] & 0xFF) < 45) && opModeIsActive()) { // black

                colorSensorLeftCache = colorSensorLeftReader.read(0x08, 1);
                colorSensorRightCache = colorSensorRightReader.read(0x08, 1);

                telemetry.addData("Left", colorSensorLeftCache[0] & 0xFF);
                telemetry.addData("Right", colorSensorRightCache[0] & 0xFF);
                telemetry.update();
            }

            setMotorPower(0.0, 0.0);
        }
    }

    public void encodersForward (double distance, double power) {

        if (opModeIsActive()) {

            motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            int DIAMETER = 4;
            int GEAR_RATIO = 1;
            int PULSES = 1120;
            double CIRCUMFERENCE = Math.PI * DIAMETER;
            double ROTATIONS = (distance / CIRCUMFERENCE) * GEAR_RATIO;
            double COUNTS = PULSES * ROTATIONS;

            COUNTS = COUNTS + Math.abs((double)motorBackRight.getCurrentPosition());

            setMotorPower(power, power);

            while ((double)motorBackRight.getCurrentPosition() < COUNTS && opModeIsActive()) {

            }

            setMotorPower(0.0, 0.0);
        }
    }

    public void encodersBackward (double distance, double power) {

        if (opModeIsActive()) {

            motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            int DIAMETER = 4;
            int GEAR_RATIO = 1;
            int PULSES = 1120;
            double CIRCUMFERENCE = Math.PI * DIAMETER;
            double ROTATIONS = (distance / CIRCUMFERENCE) * GEAR_RATIO;
            double COUNTS = PULSES * ROTATIONS;

            COUNTS = Math.abs((double)motorBackRight.getCurrentPosition()) - COUNTS;

            setMotorPower(-power, -power);

            while ((double)motorBackRight.getCurrentPosition() > COUNTS && opModeIsActive()) {

            }

            setMotorPower(0.0, 0.0);
        }
    }

    public void gyroTurnRight (int degrees, double power) {

        if (opModeIsActive()) {

            sensorGyro.resetZAxisIntegrator();

            int heading = sensorGyro.getHeading();

            setMotorPower(power, -power);

            while (heading < degrees && opModeIsActive()) {

                heading = sensorGyro.getHeading();

                if (heading >= 180) {

                    heading = 360 - heading;
                }
            }

            setMotorPower(0, 0);
        }
    }

    public void gyroTurnLeft (int degrees, double power) {

        if (opModeIsActive()) {

            sensorGyro.resetZAxisIntegrator();

            int heading = sensorGyro.getHeading();

            setMotorPower(-power, power);

            while (heading < degrees && opModeIsActive()) {

                heading = sensorGyro.getHeading();

                if (heading >= 180) {

                    heading = 360 - heading;
                }
            }

            setMotorPower(0, 0);
        }
    }

    public void shoot () {

        if (opModeIsActive()) {

            motorShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            double GEAR_RATIO = 1.456;
            int PULSES = 1680;
            double COUNTS = PULSES * GEAR_RATIO;

            COUNTS = (COUNTS + Math.abs((double)motorShooter.getCurrentPosition())) / 16.0;

            motorShooter.setPower(1.0);

            while ((double)motorShooter.getCurrentPosition() < COUNTS && opModeIsActive()) {

            }

            motorShooter.setPower(0.0);
        }
    }

    public void shoot2 () {

        if (opModeIsActive()) {

            motorShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            double GEAR_RATIO = 1.456;
            int PULSES = 1680;
            double COUNTS = PULSES * GEAR_RATIO;

            COUNTS = (COUNTS + Math.abs((double)motorShooter.getCurrentPosition())) * (3.0/4.0);

            motorShooter.setPower(1.0);

            while ((double)motorShooter.getCurrentPosition() < COUNTS && opModeIsActive()) {

            }

            motorShooter.setPower(0.0);
        }
    }

    public void shoot3 () {

        if (opModeIsActive()) {

            motorShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            double GEAR_RATIO = 1.456;
            int PULSES = 1680;
            double COUNTS = PULSES * GEAR_RATIO;

            COUNTS = (COUNTS + Math.abs((double)motorShooter.getCurrentPosition())) / 6.0;

            motorShooter.setPower(1.0);

            while ((double)motorShooter.getCurrentPosition() < COUNTS && opModeIsActive()) {

            }

            motorShooter.setPower(0.0);
        }
    }

    public void setMotorPower (double left, double right) {

        if (opModeIsActive()) {

            motorBackLeft.setPower(left);
            motorBackRight.setPower(right);
            motorFrontLeft.setPower(left);
            motorFrontRight.setPower(right);
        }
    }
}