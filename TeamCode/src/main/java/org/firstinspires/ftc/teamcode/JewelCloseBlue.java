package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.roboraiders.Robot.RoboRaidersAuto;
import com.roboraiders.Robot.Robot;

/**
 * Created by Alex and J-Dawg on 11/8/17.
 */

@Autonomous

public class JewelCloseBlue extends RoboRaidersAuto {

    public Robot robot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {

        robot.initialize(hardwareMap);
        vuforiaInitialization(hardwareMap);

        robot.setServoPosition(0.4);

        waitForStart();

        lowerArm(robot, 0.99);
        selectJewel(robot, "blue");

        encodersMove(robot, 20, 0.5, "forward");
        Thread.sleep(500);

        imuTurn(robot, 90, 0.5, "left");
        Thread.sleep(500);

        encodersMove(robot, 3, 0.5, "forward");
        Thread.sleep(500);
    }
}