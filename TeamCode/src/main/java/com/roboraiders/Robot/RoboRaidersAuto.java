package com.roboraiders.Robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

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

        telemetry.addData("Red", bot.colorSensor.red());
        telemetry.addData("Blue", bot.colorSensor.blue());
        telemetry.update();
        // does the bot need to move forward at all? or no? discuss with program team. this programs assumes no.
        //bot.servoJewel.setPosition(0.5);// lower arm with color sensor

        //assuming color sensor is mounted facing right

        //assuming red alliance

        // if (allianceColorRed == true){
        if (bot.colorSensor.red() > 675 &&  bot.colorSensor.red() <= 775) { // ball on the right is red
            //using motor power until we get math figured out for encoders

            bot.setDriveMotorPower(-1,1,1,-1); //strafe left
            Thread.sleep (500);
            bot.setDriveMotorPower(0,0,0,0);
            Thread.sleep (500);
            bot.setDriveMotorPower(1, -1, -1, 1); //strafe right to original position
            Thread.sleep (500);
        }
        else { //if the ball on the right is blue
            bot.setDriveMotorPower(1, -1, -1, 1); //strafe right
            Thread.sleep (500);
            bot.setDriveMotorPower(0,0,0,0);
            Thread.sleep (500);
            bot.setDriveMotorPower(-1,1,1,-1); //strafe left to original position
            Thread.sleep (500);

        }
        //}

        //assuming blue alliance

        //if {allianceColorRed == false){               // therefore blue alliance
        if (bot.colorSensor.blue() <= 675  && bot.colorSensor.blue() >= 575) {      // ball on the right is blue
            bot.setDriveMotorPower(-1, 1, 1, -1); //strafe left
            Thread.sleep(500);
            bot.setDriveMotorPower(0, 0, 0, 0);
            Thread.sleep(500);
            bot.setDriveMotorPower(1, -1, -1, 1); //strafe right to original position
            Thread.sleep(500);
        }
        else {
            bot.setDriveMotorPower(1, -1, -1, 1); //strafe right
            Thread.sleep (500);
            bot.setDriveMotorPower(0,0,0,0);
            Thread.sleep (500);
            bot.setDriveMotorPower(-1,1,1,-1); //strafe left to original position
            Thread.sleep (500);
        }

    }

    public void readDistance(Robot bot) {

        // loop and read the distance data.
        // note we use opModeIsActive() as our loop condition because it is an interruptable method.
        while (opModeIsActive()) {
            // send the info back to driver station using telemetry function.
            telemetry.addData("Distance (cm)",
                    String.format(Locale.US, "%.02f", bot.distanceSensor.getDistance(DistanceUnit.CM)));
            telemetry.update();
        }
    }

    public void imuTurnRight(Robot bot, float degrees, double power) {

        bot.angles = bot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        float heading = bot.angles.thirdAngle;

        bot.setDriveMotorPower(power, -power, power, -power);

        while (heading < degrees) {

            bot.angles = bot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            heading = bot.angles.thirdAngle;

            if (heading <= 0) {

                heading = 360 + heading;
            }
        }

        bot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0);
    }

    public void imuTurnLeft(Robot bot, float degrees, double power) {

        bot.angles = bot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        float heading = bot.angles.thirdAngle;

        bot.setDriveMotorPower(-power, power, -power, power);

        while (heading < degrees) {

            bot.angles = bot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            heading = bot.angles.thirdAngle;

            if (heading <= 0) {

                heading = 360 + heading;
            }
        }

        bot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0);
    }

    public void encodersStrafeRight(Robot bot, int distance, double power) {

        if (opModeIsActive()) {

            bot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            int DIAMETER = 4;
            int GEAR_RATIO = 1;
            int PULSES = 1120;
            double CIRCUMFERENCE = Math.PI * DIAMETER;
            double ROTATIONS = (distance / CIRCUMFERENCE) * GEAR_RATIO;
            double COUNTS = PULSES * ROTATIONS;

            COUNTS = COUNTS + Math.abs(bot.motorFrontLeft.getCurrentPosition());

            bot.setDriveMotorPower(power, -power, -power, power);

            while (bot.motorFrontLeft.getCurrentPosition() < COUNTS && opModeIsActive()) {

            }

            bot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0);
        }
    }

    public void encodersStrafeLeft(Robot bot, int distance, double power) {

        if (opModeIsActive()) {

            bot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            int DIAMETER = 4;
            int GEAR_RATIO = 1;
            int PULSES = 1120;
            double CIRCUMFERENCE = Math.PI * DIAMETER;
            double ROTATIONS = (distance / CIRCUMFERENCE) * GEAR_RATIO;
            double COUNTS = PULSES * ROTATIONS;

            COUNTS = Math.abs(bot.motorFrontLeft.getCurrentPosition()) - COUNTS;

            bot.setDriveMotorPower(-power, power, power, -power);

            while (bot.motorFrontLeft.getCurrentPosition() > COUNTS && opModeIsActive()) {

            }

            bot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0);
        }
    }
}
