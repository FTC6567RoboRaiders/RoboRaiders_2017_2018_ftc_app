package com.roboraiders.Robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by Alex Snyder on 10/8/17.
 */

public abstract class RoboRaidersAuto extends LinearOpMode {

    /**
     * This method is going to push the jewel off the platform that is not the current alliance color.
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
        if (bot.colorSensor.red() > 675 && bot.colorSensor.red() <= 775) { //if the ball on the right is red

            encodersStrafeLeft(bot, 6, 0.5); //strafe left
            Thread.sleep(500);

            encodersStrafeRight(bot, 6, 0.5); //strafe right to original position
            Thread.sleep(500);
        }
        else { //the ball on the right is blue

            encodersStrafeRight(bot, 6, 0.5); //strafe right
            Thread.sleep(500);

            encodersStrafeLeft(bot, 6, 0.5); //strafe left to original position
            Thread.sleep(500);
        }
        //}

        //assuming blue alliance

        //if (allianceColorRed == false){ //blue alliance
        if (bot.colorSensor.blue() <= 675 && bot.colorSensor.blue() >= 575) { //if the ball on the right is blue

            encodersStrafeLeft(bot, 6, 0.5); //strafe left
            Thread.sleep(500);

            encodersStrafeRight(bot, 6, 0.5); //strafe right to original position
            Thread.sleep(500);
        }
        else { //the ball on the right is red

            encodersStrafeRight(bot, 6, 0.5); //strafe right
            Thread.sleep (500);

            encodersStrafeLeft(bot, 6, 0.5); //strafe left to original position
            Thread.sleep (500);
        }
    }

    public void imuTurnRight(Robot bot, float degrees, double power) { //gets hardware from Robot and defines degrees as a
                                                                       //float and defines power as a double

        bot.imu.initialize(bot.parameters); //resets IMU angle to zero

        bot.angles = bot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES); //this sets up the how we want the IMU to report data
        float heading = Math.abs(bot.angles.firstAngle); //heading is equal to the absolute value of the first angle

        bot.setDriveMotorPower(power, -power, power, -power); //this defines what the power will be

        while (heading < degrees && opModeIsActive()) { //this states that while the value of heading is less then the degree value
                                                        // and while opMode is active continue the while loop

            bot.angles = bot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES); //continuous " "
            heading = Math.abs(bot.angles.firstAngle); //continuous " "

            telemetry.addData("Heading", heading); //feedback of Heading value
            telemetry.update(); //continuous update
        }

        bot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0); //stops robot
    }

    public void imuTurnLeft(Robot bot, float degrees, double power) { //same idea except going right

        bot.imu.initialize(bot.parameters);

        bot.angles = bot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        float heading = Math.abs(bot.angles.firstAngle);

        bot.setDriveMotorPower(-power, power, -power, power);

        while (heading < degrees && opModeIsActive()) {

            bot.angles = bot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            heading = Math.abs(bot.angles.firstAngle);

            telemetry.addData("Heading", heading);
            telemetry.update();
        }

        bot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0);
    }

    public void encodersStrafeRight(Robot bot, int distance, double power) { //sets parameters for this method

        if (opModeIsActive()) { //while active

            bot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //uses encoders for front left wheel
            bot.motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //uses encoder for front right wheel
            bot.motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //uses encoder for back left wheel
            bot.motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //uses encoders for back right wheel

            int DIAMETER = 4; //diameter of wheel
            int GEAR_RATIO = 1; //gear ratio
            int PULSES = 1120; //encoder counts in one revolution
            double CIRCUMFERENCE = Math.PI * DIAMETER; //gives you circumference
            double ROTATIONS = (distance / CIRCUMFERENCE) * GEAR_RATIO; //gives the rotations
            double COUNTS = PULSES * ROTATIONS; //gives the counts

            COUNTS = COUNTS + Math.abs(bot.motorFrontLeft.getCurrentPosition()); //add desired counts to
                                                                                 //current counts to strafe left

            bot.setDriveMotorPower(power, -power, -power, power);

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

    public void encodersStrafeLeft(Robot bot, int distance, double power) { //sets parameters for this method

        if (opModeIsActive()) { //while active

            bot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //uses encoders for front left wheel
            bot.motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //uses encoders for front right wheel
            bot.motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //uses encoders for back left wheel
            bot.motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //uses encoders for back right wheel

            int DIAMETER = 4; //diameter of wheel
            int GEAR_RATIO = 1; //gear ratio
            int PULSES = 1120; //how many encoder counts in one revolution
            double CIRCUMFERENCE = Math.PI * DIAMETER; //this gives you circumference
            double ROTATIONS = (distance / CIRCUMFERENCE) * GEAR_RATIO; //give the rotations
            double COUNTS = PULSES * ROTATIONS; //give the counts

            COUNTS = Math.abs(bot.motorFrontLeft.getCurrentPosition()) - COUNTS; // subtracts desired counts by the current count to strafe left

            bot.setDriveMotorPower(-power, power, power, -power);

            while (bot.motorFrontLeft.getCurrentPosition() > COUNTS && opModeIsActive()) {

                telemetry.addData("COUNTS", COUNTS); //shows amount of counts on phone
                telemetry.update(); //continuously updates counts above

                telemetry.addData("Front Left", bot.motorFrontLeft.getCurrentPosition());
                telemetry.addData("Front Right", bot.motorFrontRight.getCurrentPosition());
                telemetry.addData("Back Left", bot.motorBackLeft.getCurrentPosition());
                telemetry.addData("Back Right", bot.motorBackRight.getCurrentPosition());
                telemetry.update();

            }

            bot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0);
        }
    }

    public void touchSensorCount(Robot bot, int wallsTarget, double power) { //establishes parameters
                                                                             //for method

        bot.setDriveMotorPower(power, -power, -power, power); //robot is moving at whatever power is specified

        while (bot.wallsTouch < wallsTarget && opModeIsActive()) {

            bot.currStateTouch = bot.digitalTouch.getState();

            if (bot.digitalTouch.getState()) { //a true is returned from getState() means that the
                                               //button is not being pressed

                telemetry.addData("Digital Touch", "Is Not Pressed");
                telemetry.update();
            }
            else { //a false returned from getState() means that the button is being pressed

                telemetry.addData("Digital Touch", "Is Pressed");
                telemetry.update();
            }

            if (!bot.currStateTouch && bot.currStateTouch != bot.prevStateTouch) { //if the robot is touching the wall
                //(if the current state is true and the current
                //state is not equal to the previous state)
                //Anyway, if the touch sensor is just starting to be pressed:

                bot.wallsTouch++; //add 1 to the current "wallsTouch" variable
                bot.prevStateTouch = bot.currStateTouch; //now the previous state is the same as the current state
            }
            else if (bot.currStateTouch && bot.currStateTouch != bot.prevStateTouch) { //if the touch
                //sensor is just starting to not be pressed:

                bot.prevStateTouch = bot.currStateTouch; //now the previous state equals the current state,
                //don't change anything to the "wallsTouch" variable
            }
        }

        bot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0); //stop the robot
    }

    public void distanceSensorCount(Robot bot, int wallsTarget, double power) { //establishes
                                                                              //parameters for method

        bot.setDriveMotorPower(power, -power, -power, power); //robot is moving at whatever power is specified

        while (bot.wallsDistance < wallsTarget && opModeIsActive()) { //while the robot has not yet hit the specified number of walls

            if (bot.distanceSensor.getDistance(DistanceUnit.CM) <= 20) { //if the distance of the
                //sensor is less than the
                //pre-specified value, aka the robot is passing
                //close to the wall

                bot.currStateDistance = true; //the robot is currently passing a wall
                telemetry.addData("Distance Sensor", "Is In Front of a Wall");
                telemetry.update();
            }
            else { //if the distance of the sensor is greater than the
                //pre-specified value, aka the robot is between walls

                bot.currStateDistance = false; //the robot is not currently passing a wall
                telemetry.addData("Digital Sensor", "Is Not In Front of a Wall");
                telemetry.update();
            }

            if (bot.currStateDistance && bot.currStateDistance != bot.prevStateDistance) { //if the robot sees the
                //wall and it didn't see the wall before
                //basically, if the robot sees the wall

                bot.wallsDistance++; // add 1 to the current "wallsDistance" variable
                bot.prevStateDistance = bot.currStateDistance; //now the previous state is the same as the current state
            }
            else if (!bot.currStateDistance && bot.currStateDistance != bot.prevStateDistance) { //if the touch sensor
                // is just starting to not be pressed:

                bot.prevStateDistance = bot.currStateDistance; //now the previous state equals the current state,
                //don't change anything to the "wallsDistance" variable
            }
        }

        bot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0); //stop the robot
    }
}
