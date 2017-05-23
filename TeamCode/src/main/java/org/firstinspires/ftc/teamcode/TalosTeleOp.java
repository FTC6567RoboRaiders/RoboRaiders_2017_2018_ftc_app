package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Katelin Zichittella on 11/1/2016.
 */

@TeleOp

public class TalosTeleOp extends OpMode {

    DcMotor motorBackLeft, motorBackRight, motorFrontLeft, motorFrontRight,
            motorShooter, motorSweeper, motorLift1, motorLift2;
    Servo servoBeacon, servoGate, servoLiftLeft, servoLiftRight;
    GyroSensor sensorGyro;

    double motorFactor = 1.0;
    double sweeperMode;
    boolean gamepad2_a_currState = false;
    boolean gamepad2_a_prevState = false;
    boolean gamepad2_b_currState = false;
    boolean gamepad2_b_prevState = false;

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

    @Override
    public void start() {

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

        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorSweeper.setDirection(DcMotor.Direction.REVERSE);
        servoBeacon.setPosition(0.0);
        servoGate.setPosition(0.0);
        servoLiftLeft.setPosition(0.70);
        servoLiftRight.setPosition(0.67);
        motorFactor = 1.0;
        sweeperMode = 0.0;
    }

    @Override
    public void init() {

    }

    @Override
    public void loop() {

        float left = gamepad1.left_stick_y;
        float right = gamepad1.right_stick_y;
        float shoot = gamepad2.right_stick_y;
        float lift = gamepad2.left_stick_y;

        left = Range.clip(left, -1, 1);
        right = Range.clip(right, -1, 1);
        shoot = Range.clip(shoot, -1, 1);
        lift = Range.clip(lift, -1, 1);

        left = (float) scaleInput(left);
        right = (float) scaleInput(right);
        shoot = (float) scaleInput(shoot);
        lift = (float) scaleInput(lift);

        setMotorPower(left * motorFactor, right * motorFactor);
        setAttachmentPower(sweeperMode, shoot, lift);

        gamepad2_a_currState = gamepad2.a;
        gamepad2_b_currState = gamepad2.b;

        if (gamepad1.x) {

            motorFactor = 0.5;
        }

        if (gamepad1.y) {

            motorFactor = 1.0;
        }

        if (gamepad1.a) {

            motorFactor = -0.5;
        }

        if (gamepad1.b) {

            motorFactor = -1.0;
        }

        if (gamepad1.left_bumper) {

            servoBeacon.setPosition(1);
        }

        if (gamepad1.right_bumper) {

            servoBeacon.setPosition(0);
        }

        if (gamepad2_a_currState &&
                gamepad2_a_currState != gamepad2_a_prevState)  {

            servoLiftLeft.setPosition(0.70);
            servoLiftRight.setPosition(0.67);

            gamepad2_a_prevState = gamepad2_a_currState;
        }
        else if (gamepad2_a_currState == false &&
                gamepad2_a_currState != gamepad2_a_prevState)  {

            gamepad2_a_prevState = gamepad2_a_currState;
        }

        if (gamepad2_b_currState &&
                gamepad2_b_currState != gamepad2_b_prevState)  {

            servoLiftLeft.setPosition(0.35);
            servoLiftRight.setPosition(0.32);

            gamepad2_b_prevState = gamepad2_b_currState;
        }
        else if (gamepad2_b_currState == false &&
                gamepad2_b_currState != gamepad2_b_prevState)  {

            gamepad2_b_prevState = gamepad2_b_currState;
        }

        if (gamepad2.dpad_left) {

            servoLiftLeft.setPosition(0.29);
            servoLiftRight.setPosition(0.67);
        }

        if (gamepad2.dpad_right) {

            servoLiftLeft.setPosition(0.70);
            servoLiftRight.setPosition(0.26);
        }

        if (gamepad2.right_bumper) {

            sweeperMode = 1.0;
        }
        else if (gamepad2.left_bumper) {

            sweeperMode = -1.0;
        }
        else {

            sweeperMode = 0.0;
        }

        if (gamepad2.dpad_down) {

            servoGate.setPosition(0.3);
        }
        else {

            servoGate.setPosition(0.0);
        }
    }

    @Override
    public void stop() {


    }

    public void setMotorPower (double left, double right) {

        motorBackLeft.setPower(left);
        motorBackRight.setPower(right);
        motorFrontLeft.setPower(left);
        motorFrontRight.setPower(right);
    }

    public void setAttachmentPower (double sweeperMode, double shoot, double lift) {

        motorSweeper.setPower(sweeperMode);
        motorShooter.setPower(shoot);
        motorLift1.setPower(lift);
        motorLift2.setPower(lift);
    }

    double scaleInput(double dVal) {

        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

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

        // Now remember a negative value might have been passed to this method
        // So return the favor and make the scaled output negative if the method
        // originally passed a negative value
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        // return scaled value.
        return dScale;
    }
}