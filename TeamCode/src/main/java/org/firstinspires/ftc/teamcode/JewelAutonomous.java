package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.roboraiders.Robot.Robot;

/**
 * Created by Nick Urbin and Kevin McCrudden on 10/1/17.
 */

@Autonomous
@Disabled

public class JewelAutonomous extends LinearOpMode {

    public Robot robot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {

        while (opModeIsActive()) {

          /*  robot.sensorColor.red();
            robot.sensorColor.blue();

            telemetry.addData("Red", robot.sensorColor.red());
            telemetry.addData("Blue", robot.sensorColor.blue());
            telemetry.update();*/
        }
    }
}