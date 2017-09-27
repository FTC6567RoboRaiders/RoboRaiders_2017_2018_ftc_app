package com.roboraiders.Robot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by Alex Snyder on 9/27/17.
 */

@Autonomous

public class ColorSensorAutonomous extends LinearOpMode {

    public Robot robot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {

        while (opModeIsActive()) {

            robot.sensorColor.red();
            robot.sensorColor.blue();

            telemetry.addData("Red", robot.sensorColor.red());
            telemetry.addData("Blue", robot.sensorColor.blue());
            telemetry.update();
        }
    }
}