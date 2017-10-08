package com.roboraiders.Robot;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

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
    
    public Servo servoJewel = null;

    public ColorSensor colorSensor;
    public DistanceSensor distanceSensor;
    public ModernRoboticsI2cRangeSensor rangeSensor;
    /*public I2cDeviceSynch rangeSensorReader;
    public byte[] rangeSensorCache;*/
    public BNO055IMU imu;

    /* Local OpMode Members */
    HardwareMap hwMap =  null;

    /* Public Variables */
    public String pictograph;
    public BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    public Orientation angles;

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
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotor.Direction.FORWARD);
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.FORWARD);

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
        rangeSensor = hwMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range");
        imu = hwMap.get(BNO055IMU.class, "imu");
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
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


    /** moveUntilWall will move the robot until it reaches a defined distance from a barrier
     *
     * <br>
     * <b>Note:</b> This method we may want to move to a Drive class that will handle all of
     * the "heavy" work of moving and positioning the robot during autonomous, the team will
     * need to discuss how we want to organize this
     * <br>
     * <b>Author(s):</b> Alex Synder, Katelin Zichittella
     * <br>
     *
     *
     *
     *
     *
     */
    /*@param distance the distance from the wall that the robot should be away from a barrier
     *                 or in this case the field perimeter wall */

    public void imuTurnLeft(float degrees, double power) {

        float heading = angles.firstAngle;

        setDriveMotorPower(-power, power, -power, power);

        while (heading < degrees) {

            if (heading >= 180) {

                heading = 360 - heading;
            }
        }

        setDriveMotorPower(-0.0, 0.0, -0.0, 0.0);
    }

    public void imuTurnRight(float degrees, double power) {

        float heading = angles.firstAngle;

        setDriveMotorPower(power, -power, power, -power);

        while (heading < degrees) {

            if (heading >= 180) {

                heading = 360 - heading;
            }
        }

        setDriveMotorPower(-0.0, 0.0, -0.0, 0.0);
    }
}
