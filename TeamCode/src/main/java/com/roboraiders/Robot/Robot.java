package com.roboraiders.Robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
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
    public VuforiaLocalizer vuforia;
    public VuforiaTrackable relicTemplate;
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
    public boolean currStateTouch = false;
    public  getTouchState () {
        currStateTouch = digitalTouch.getState();

    }
    public void runWithEncoders ()    {
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //uses encoders for front left wheel
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //uses encoder for front right wheel
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //uses encoder for back left wheel
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //uses encoders for back right wheel
    }
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
        // May want to use RUN_USING_ENCODER if encoders are installed, and we wouldn't use encoders for teleop, even if we
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //will use them in teleop.
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
        digitalTouch = hwMap.get(DigitalChannel.class, "sensor_touch");
        digitalTouch.setMode(DigitalChannel.Mode.INPUT);

        // Vuforia initialization
        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AedUDNP/////AAAAGXH2ZpUID0KanSX9ZSR37LKFSFokxIqmy/g0BNepdA9EepixxnO00qygLnMJq3Fg9gZxnkUJaKgk14/UjhxPWVQIs90ZXJLc21NvQvOeZ3dOogagVP8yFnFQs2xCijGmC/CE30ojlAnbhAhqz1y4tZPW2QkK5Qt0xCakTTSAw3KPQX2mZxX+qMxI2ljrN0eaxaKVnKnAUl8x3naF1mez7f9c8Xdi1O5auL0ePdG6bJhWjEO1YwpSd8WkSzNDEkmw20zpQ7zaOOPw5MeUQUr9vAS0fef0GnLjlS1gb67ajUDlEcbbbIeSrLW/oyRGTil8ueQC2SWafdspSWL3SJNaQKWydies23BxJxM/FoLuYYjx";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");
        relicTrackables.activate();
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
