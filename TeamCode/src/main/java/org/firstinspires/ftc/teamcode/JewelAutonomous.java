package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.roboraiders.Robot.RoboRaidersAuto;
import com.roboraiders.Robot.Robot;


/**
 * Created by (mostly) Nick Urbin and (a little bit) Kevin McCrudden on 10/1/17.
 */

@Autonomous

public class JewelAutonomous extends RoboRaidersAuto {

    int red = 1;
    int blue = 2;

    public Robot robot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {

        robot.initialize(hardwareMap);
        //robot.servoJewel.setPosition(0.0);

        waitForStart();

        robot.colorSensor.red();
        robot.colorSensor.blue();

        selectJewel(robot, red);
    }
}


