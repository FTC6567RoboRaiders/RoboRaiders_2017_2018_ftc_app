package com.roboraiders.Robot;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

/**
 * Created by Alex Snyder and Nick Urbin (and a little bit from Kevin Mccrudden) on 10/8/17.
 */

public abstract class RoboRaiderAuto extends LinearOpMode {

    /**
     * This program is going to push the jewel off the platform that isn't the current alliance color.
     *
     * @param Bot the Bot currently being worked on
     * @param allianceColor the color of your alliance
     */

    public void SelectJewel(Robot Bot, int allianceColor) throws InterruptedException {

        telemetry.addData("Red", Bot.colorSensor.red());
        telemetry.addData("Blue", Bot.colorSensor.blue());
        telemetry.update();
        // does the Bot need to move forward at all? or no? discuss with program team. this programs assumes no.
        Bot.servoJewel.setPosition(0.5);// lower arm with color sensor

        //assuming color sensor is mounted facing right

        //assuming red alliance

        // if (allianceColorRed == true){
        if (Bot.colorSensor.red() > 675 &&  Bot.colorSensor.red() <= 775) { // ball on the right is red
            //using motor power until we get math figured out for encoders

            Bot.setDriveMotorPower(-1,1,1,-1); //strafe left
            Thread.sleep (500);
            Bot.setDriveMotorPower(0,0,0,0);
            Thread.sleep (500);
            Bot.setDriveMotorPower(1, -1, -1, 1); //strafe right to original position
            Thread.sleep (500);
        }
        else { //if the ball on the right is blue
            Bot.setDriveMotorPower(1, -1, -1, 1); //strafe right
            Thread.sleep (500);
            Bot.setDriveMotorPower(0,0,0,0);
            Thread.sleep (500);
            Bot.setDriveMotorPower(-1,1,1,-1); //strafe left to original position
            Thread.sleep (500);

        }
        //}

        //assuming blue alliance

        //if {allianceColorRed == false){               // therefore blue alliance
        if (Bot.colorSensor.blue() <= 675  && Bot.colorSensor.blue() >= 575) {      // ball on the right is blue
            Bot.setDriveMotorPower(-1, 1, 1, -1); //strafe left
            Thread.sleep(500);
            Bot.setDriveMotorPower(0, 0, 0, 0);
            Thread.sleep(500);
            Bot.setDriveMotorPower(1, -1, -1, 1); //strafe right to original position
            Thread.sleep(500);
        }
        else {
            Bot.setDriveMotorPower(1, -1, -1, 1); //strafe right
            Thread.sleep (500);
            Bot.setDriveMotorPower(0,0,0,0);
            Thread.sleep (500);
            Bot.setDriveMotorPower(-1,1,1,-1); //strafe left to original position
            Thread.sleep (500);
        }

    }



}
