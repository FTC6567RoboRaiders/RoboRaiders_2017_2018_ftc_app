package com.roboraiders.Robot;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * This is NOT an Op Mode.
 *
 * This class will be used to define all functions for our 2017 - 2018 robot.
 *
 */

public class Robot {

    /* Robot Motors */
    public DcMotor motorFrontLeft = null;
    public DcMotor motorFrontRight = null;
    public DcMotor motorBackLeft = null;
    public DcMotor motorBackRight = null;
    public Servo servoJewel = null;
    public ColorSensor sensorColor;
    public DistanceSensor sensorDistance;
    public ModernRoboticsI2cRangeSensor rangeSensor;

    I2cDeviceSynch rangeSensorReader;
    byte[] rangeSensorCache;


    /* Local OpMode Members */
    HardwareMap hwMap =  null;

    /* Variables */
    public String pictograph;

    /* Constructor */
    public Robot(){

    }

    /**  init - initialize the robot
     *
     *
     *
     * @param ahwMap - hardware map for the robot
     *
     */

    public void initialize(HardwareMap ahwMap) {

        // Save reference to hardware map
        hwMap = ahwMap;

        // Define and initialize motors
        motorFrontLeft = hwMap.get(DcMotor.class, "left_Front");
        motorFrontRight = hwMap.get(DcMotor.class, "right_Front");
        motorBackLeft = hwMap.get(DcMotor.class, "left_Back");
        motorBackRight = hwMap.get(DcMotor.class, "right_Back");

        // Defines the directions the motors will spin
        motorFrontLeft.setDirection(DcMotor.Direction.FORWARD);
        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotor.Direction.FORWARD);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set all motors to zero power
        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackRight.setPower(0);
        motorBackLeft.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize servos
        servoJewel = hwMap.get(Servo.class, "servo_Jewel");

        // Define and initialize sensors
        sensorColor = hwMap.get(ColorSensor.class, "sensor_color_distance");
        sensorDistance = hwMap.get(DistanceSensor.class, "sensor_color_distance");
        rangeSensor = hwMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range");
    }

    /** setDriveMotorPower
     *
     * @param leftFront
     * @param rightFront
     * @param leftBack
     * @param rightBack
     */

    public void setDriveMotorPower(double leftFront, double rightFront, double leftBack, double rightBack){

        motorFrontLeft.setPower(leftFront);
        motorFrontRight.setPower(rightFront);
        motorBackLeft.setPower(leftBack);
        motorBackRight.setPower(rightBack);
    }

    public void moveUntilWall (double distance) {

        setDriveMotorPower(0.24, 0.24, 0.24, 0.24); // ...set all of the motors to a positive speed of 0.24...

        rangeSensorCache = rangeSensorReader.read(0x04, 1);

        while ((rangeSensorCache[0] & 0xFF) > distance) {

            rangeSensorCache = rangeSensorReader.read(0x04, 1);
        }

        setDriveMotorPower(0.0, 0.0, 0.0, 0.0); // "Once the desired distance away from the barrier is
        // reached, stop the robot."
    }
}
