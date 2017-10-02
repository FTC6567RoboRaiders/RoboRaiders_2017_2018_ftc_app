package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.roboraiders.Robot.Robot;


/**
 * Created by Alex Snyder and Fiona Beer on 10/1/2017.
 */

@Autonomous
@Disabled

public class CryptoboxAutonomous extends LinearOpMode {

    public Robot robot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {

        robot.initialize(hardwareMap); //sets parameters for initialize sequence

        waitForStart();

        robot.moveUntilWall(6); //robot moves to wall until it is six inches away.
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
    /* public void moveUntilWall (double distance) {

        robot.setDriveMotorPower(0.24, 0.24, 0.24, 0.24); // ...set all of the motors to a positive speed of 0.24...

        rangeSensorCache = rangeSensorReader.read(0x04, 1);

        while ((rangeSensorCache[0] & 0xFF) > distance) {

            rangeSensorCache = rangeSensorReader.read(0x04, 1);
        }

        setDriveMotorPower(0.0, 0.0, 0.0, 0.0); // "Once the desired distance away from the barrier is
        // reached, stop the robot."
    }*/

}
