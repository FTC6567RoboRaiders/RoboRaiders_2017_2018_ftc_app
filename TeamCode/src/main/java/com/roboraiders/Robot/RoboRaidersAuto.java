package com.roboraiders.Robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


/**
 * Created by Alex Snyder on 10/8/17.  Comment update so alex can pick up changes
 */

public abstract class RoboRaidersAuto extends LinearOpMode {


    public int dividersTouch = 0; //counts the number of times that the robot hits the wall with the touch sensor
    public double dividersDistance = 0; //counts the number of times that the robot hits the wall with the distance sensor
    public boolean currStateTouch = false;
    public boolean prevStateTouch = false;
    public boolean currStateDistance = false;
    public boolean prevStateDistance = false;



    public VuforiaLocalizer vuforia;
    public VuforiaTrackable relicTemplate;
    public String pictograph;

    /**
     * This method is going to push the jewel off the platform that is not the current alliance color
     *
     * @param bot the bot currently being worked on
     * @param allianceColor the color of your alliance
     */
    public void selectJewel(Robot bot, int allianceColor) throws InterruptedException {

        //Does the robot need to move forward at all? Or no? Discuss with programming team. This program assumes no.
        //assuming color sensor is mounted facing right

        //bot.servoJewel.setPosition(0.5); //lower arm with color sensor

        //assuming red alliance

        //if (allianceColorRed == true){ //red alliance
        if (bot.getColorIntensity("red") > 675 && bot.getColorIntensity("red") <= 775) { //if the ball on the right is red

            encodersStrafe(bot, 6, 0.5, "left"); //strafe left
            Thread.sleep(500);

            encodersStrafe(bot, 6, 0.5, "right"); //strafe right to original position
            Thread.sleep(500);
        }
        else { //the ball on the right is blue

            encodersStrafe(bot, 6, 0.5, "right"); //strafe right
            Thread.sleep(500);

            encodersStrafe(bot, 6, 0.5, "left"); //strafe left to original position
            Thread.sleep(500);
        }
        //}

        //assuming blue alliance

        //if (allianceColorRed == false){ //blue alliance
        if (bot.getColorIntensity("blue") <= 675 && bot.getColorIntensity("blue") >= 575) { //if the ball on the right is blue

            encodersStrafe(bot, 6, 0.5, "left"); //strafe left
            Thread.sleep(500);

            encodersStrafe(bot, 6, 0.5, "right"); //strafe right to original position
            Thread.sleep(500);
        }
        else { //the ball on the right is red

            encodersStrafe(bot, 6, 0.5, "right"); //strafe right
            Thread.sleep (500);

            encodersStrafe(bot, 6, 0.5, "left"); //strafe left to original position
            Thread.sleep (500);
        }
    }

    /**
     * This method will turn the robot right or left a certain angle measure using the IMU
     *
     * @param bot the bot currently being worked on
     * @param degrees the desired number of degrees to turn
     * @param power the desired power the wheel motors will run at
     * @param direction the direction the robot is turning; either right or left.
     */
    public void imuTurn(Robot bot, float degrees, double power, String direction) { //gets hardware from Robot and defines degrees as a
                                                                       //float and defines power as a double, and direction as a string

        bot.resetIMU(); //resets IMU angle to zero

        bot.angles = bot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES); //this sets up the how we want the IMU to report data
        float heading = Math.abs(bot.angles.firstAngle); //heading is equal to the absolute value of the first angle


        if (direction.equals("right")) {
            bot.setDriveMotorPower(power, -power, power, -power); // the robot will turn right
        }
        else if(direction.equals("left")){
            bot.setDriveMotorPower(-power, power, -power, power); // the robot will turn left
        }

        while (heading < degrees && opModeIsActive()) { //this states that while the value of heading is less then the degree value
                                                        // and while opMode is active continue the while loop

            bot.angles = bot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES); //continuous " "
            heading = Math.abs(bot.angles.firstAngle); //continuous " "

            telemetry.addData("Heading", heading); //feedback of Heading value
            telemetry.update(); //continuous update
        }

        bot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0); //stops robot
    }


    /**
     * This method will strafe the robot either left or right a certain distance in inches using encoders
     *
     * @param bot the bot currently being worked on
     * @param distance the desired distance the robot will travel
     * @param power the desired power the wheel motors will run at
     * @param direction the direction the robot is strafing: either right or left
     */
    public void encodersStrafe(Robot bot, int distance, double power, String direction) { //sets parameters for this method

        if (opModeIsActive()) { //while active

            bot.runWithEncoders(); // Takes method in robot and says to use encoders here

            int DIAMETER = 4; //diameter of wheel
            int GEAR_RATIO = 1; //gear ratio
            int PULSES = 1120; //encoder counts in one revolution
            double CIRCUMFERENCE = Math.PI * DIAMETER; //gives you circumference
            double ROTATIONS = (distance / CIRCUMFERENCE) * GEAR_RATIO; //gives the rotations
            double COUNTS = PULSES * ROTATIONS; //gives the counts


            if (direction.equals("right")) {

                COUNTS = COUNTS + Math.abs(bot.motorFrontLeft.getCurrentPosition()); //add desired counts to
                                                                                     //current counts to strafe left
                bot.setDriveMotorPower(power, -power, -power, power); //makes the robot go right

            }

            else if(direction.equals("left")) {

                COUNTS = Math.abs(bot.motorFrontLeft.getCurrentPosition()) - COUNTS; // subtracts desired counts by the current count to strafe left

                bot.setDriveMotorPower(-power, power, power, -power); //makes the robot go left

            }

            while (bot.motorFrontLeft.getCurrentPosition() < COUNTS && opModeIsActive()) {


                telemetry.addData("COUNTS", COUNTS); //shows counts on phone
                telemetry.update(); //continuously updates the counts

                telemetry.addData("Front Left", bot.motorFrontLeft.getCurrentPosition());
                telemetry.addData("Front Right", bot.motorFrontRight.getCurrentPosition());
                telemetry.addData("Back Left", bot.motorBackLeft.getCurrentPosition());
                telemetry.addData("Back Right", bot.motorBackRight.getCurrentPosition());
                telemetry.update();

            }

            bot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0);
        }
    }

    /**
     * This method will strafe the robot right until the touch sensor has detected the robot has
     * passed a certain number of dividers
     *
     * @param bot the bot currently being worked on
     * @param dividersTarget the desired number of dividers to pass
     * @param power the desired power the wheel motors will run at
     */
    public void touchSensorCount(Robot bot, int dividersTarget, double power) { //establishes parameters for method
                                                                                //and the opMode has not been stopped


        bot.setDriveMotorPower(power, -power, -power, power); //robot is moving at whatever power is specified

        while (dividersTouch < dividersTarget && opModeIsActive()) {

            bot.getTouchState();
            if (bot.digitalTouch.getState()) { //a true is returned from getState() means that the
                                               //button is not being pressed

                telemetry.addData("Digital Touch", "Is Not Pressed");
                telemetry.update();
            }
            else { //a false returned from getState() means that the button is being pressed

                telemetry.addData("Digital Touch", "Is Pressed");
                telemetry.update();
            }

            if (!bot.currStateTouch && bot.currStateTouch != prevStateTouch) { //if the robot is touching the divider
                //(if the current state is true and the current
                //state is not equal to the previous state)
                //Anyway, if the touch sensor is just starting to be pressed:

               dividersTouch++; //add 1 to the current "dividersTouch" variable
               prevStateTouch = currStateTouch; //now the previous state is the same as the current state
            }
            else if (currStateTouch && currStateTouch != prevStateTouch) { //if the touch
                //sensor is just starting to not be pressed:

                prevStateTouch = currStateTouch; //now the previous state equals the current state,
                //don't change anything to the "dividersTouch" variable
            }
        }

        bot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0); //stop the robot
    }

    /**
     * This method will strafe the robot right until the distance sensor has detected the robot has
     * passed a certain number of dividers
     *
     * @param bot the bot currently being worked on
     * @param dividersTarget the desired number of dividers to pass
     * @param power the desired power the wheel motors will run at
     * @param desiredDistance the desired distance from the target
     */
    public void distanceSensorCount(Robot bot, int dividersTarget, double power, int desiredDistance) { //establishes
                                                                              //parameters for method

        bot.setDriveMotorPower(power, -power, -power, power); //robot is moving at whatever power is specified

        while (dividersDistance < dividersTarget && opModeIsActive()) { //while the robot has not yet hit the specified number of dividers
                                                                        //and the opMode has not been stopped

            if (bot.distanceSensor.getDistance(DistanceUnit.CM) <= desiredDistance) { //if the distance of the
                //sensor is less than the
                //pre-specified value, aka the robot is passing
                //close to the divider

                currStateDistance = true; //the robot is currently passing a divider
                telemetry.addData("Distance Sensor", "Is In Front of a Divider");
                telemetry.update();
            }
            else { //if the distance of the sensor is greater than the
                   //pre-specified value, aka the robot is between dividers

                currStateDistance = false; //the robot is not currently passing a divider
                telemetry.addData("Digital Sensor", "Is Not In Front of a Divider");
                telemetry.update();
            }

            if (currStateDistance && currStateDistance != prevStateDistance) { //if the robot sees the
                //divider and it didn't see the divider before
                //basically, if the robot sees the divider

                dividersDistance++; // add 1 to the current "dividersDistance" variable
                prevStateDistance = currStateDistance; //now the previous state is the same as the current state
            }
            else if (!currStateDistance && currStateDistance != prevStateDistance) { //if the touch sensor
                //is just starting to not be pressed:

                prevStateDistance = currStateDistance; //now the previous state equals the current state,
                //don't change anything to the "dividersDistance" variable
            }
        }

        bot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0); //stop the robot
    }

    /**
     * This program initializes vuforia.
     *
     * @param bot the bot currently being worked on
     * @param hwMap hardware map
     */

    public void vuforiaInitialization(Robot bot, HardwareMap hwMap){

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

    /**
     * this program shows telemetry for which pictograph is being viewed by the phone.
     *
     * @return
     */

    public String getRelicRecoveryVuMark(){

        String pictograph;
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

        if (vuMark.equals(RelicRecoveryVuMark.LEFT)) {

            pictograph = "LEFT";
        }
        else if (vuMark.equals(RelicRecoveryVuMark.CENTER)) {

            pictograph = "CENTER";
        }
        else if (vuMark.equals(RelicRecoveryVuMark.RIGHT)) {

            pictograph = "RIGHT";
        }
        else {

            pictograph = "UNKNOWN";
        }

        return pictograph;


    }

}
