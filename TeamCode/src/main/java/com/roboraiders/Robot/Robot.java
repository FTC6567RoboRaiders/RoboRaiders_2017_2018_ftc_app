package com.roboraiders.Robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * This is NOT an Op Mode.
 *
 * This class will be used to define all "basic" functions for our 2017 - 2018 robot.
 * <br>
 * Basic functions would include things like:
 * <ul>
 *     <li>Setting power to motors</li>
 *     <li>Obtaining color sensor information</li>
 *     <li>Obtaining distance sensor information</li>
 *     <li>Obtaining encoder counts</li>
 *     <li>Setting servo position(s)</li>
 * </ul>
 * <br>
 * Any advanced functionality, lets say like <u>following a white line</u> or <u>moving the
 * robot until a given distance from a barrier</u> should be handled in a different class
 * (e.g. a Driver class).
 * <br>
 * <b>Author(s):</b> Jason Sember, Alex Snyder, Katelin Zichittella, add your name here ...
 *
 */

public class Robot {

    /* Robot Motors, Servos, and Sensors */
    public DcMotor motorFrontLeft = null;
    public DcMotor motorFrontRight = null;
    public DcMotor motorBackLeft = null;
    public DcMotor motorBackRight = null;
    
    //public Servo servoJewel = null;

    public ColorSensor colorSensor;
    public DistanceSensor distanceSensor;
    public BNO055IMU imu;
    public DigitalChannel digitalTouch;

    /* Local OpMode Members */
    public HardwareMap hwMap =  null;

    /* Public Variables */
    public String pictograph;
    public BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    public Orientation angles;
    public int wallsTouch = 0;
    public double wallsDistance = 0;
    public boolean currStateTouch = false;
    public boolean prevStateTouch = false;
    public boolean currStateDistance = false;
    public boolean prevStateDistance = false;

    /** Constructor for Robot class, current does nothing but is needed since every class needs a constructor
     *
     */
    public Robot(){

    }

    /**  init - initialize the robot
     * <br>
     * <b>Author(s):</b> Jason Sember
     * <br>
     * @param ahwMap - hardware map for the robot
     *
     *
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
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);

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
        //servoJewel = hwMap.get(Servo.class, "servo_Jewel");

        // Define and initialize sensors
        colorSensor = hwMap.get(ColorSensor.class, "sensor_color_distance");
        distanceSensor = hwMap.get(DistanceSensor.class, "sensor_color_distance");
        imu = hwMap.get(BNO055IMU.class, "imu");
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);
        digitalTouch = hwMap.get(DigitalChannel.class, "sensor_digital");
        digitalTouch.setMode(DigitalChannel.Mode.INPUT);

        // Vuforia initialization
    }

    /** setDriveMotorPower sets the power for the drive motors
     * <br>
     * <b>Author(s):</b> Jason Sember
     * <br>
     * @param leftFront power setting for the left front motor
     * @param rightFront power setting for the right front motor
     * @param leftBack power setting for the left back motor
     * @param rightBack power setting for the right back motor
     */
    public void setDriveMotorPower(double leftFront, double rightFront, double leftBack, double rightBack){

        motorFrontLeft.setPower(leftFront);
        motorFrontRight.setPower(rightFront);
        motorBackLeft.setPower(leftBack);
        motorBackRight.setPower(rightBack);
    }
}
