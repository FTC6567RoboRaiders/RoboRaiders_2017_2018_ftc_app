package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.roboraiders.Robot.Robot;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by Alex Snyder and Fiona Beer on 10/1/2017.
 */

@Autonomous
//@Disabled

public class CryptoboxAutonomous extends LinearOpMode {

    public Robot robot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {

        robot.initialize(hardwareMap); //sets parameters for initialize sequence0

        waitForStart();

        // Move until wall
        robot.setDriveMotorPower(0, 0, 0, 0);

        robot.setDriveMotorPower(1, 1, 1, 1); //move forward for two seconds
        Thread.sleep(2000);
        robot.setDriveMotorPower(0, 0, 0, 0);
        Thread.sleep(500);

        robot.setDriveMotorPower(1, -1, -1, 1); //robot strafes right
        Thread.sleep(1000);
        robot.setDriveMotorPower(0, 0, 0, 0);
        Thread.sleep(500);
    }
}
