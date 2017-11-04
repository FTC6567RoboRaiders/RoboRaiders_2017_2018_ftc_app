package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.roboraiders.Robot.RoboRaidersAuto;
import com.roboraiders.Robot.Robot;

/**
 * Created by Nick on 10/8/17.
 */

@Autonomous

public class SensorTest extends RoboRaidersAuto {

    public Robot robot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {

        robot.initialize(hardwareMap);
        telemetry.addData("Initialized", true);
        telemetry.update();

        waitForStart();

        //while (opModeIsActive()) {

            /*robot.colorSensor.red();
            robot.colorSensor.blue();
            robot.distanceSensor.getDistance(DistanceUnit.CM);
            robot.digitalTouch.getState();
            robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            getRelicRecoveryVuMark()

            telemetry.addData("Red", robot.colorSensor.red());
            telemetry.addData("Blue", robot.colorSensor.blue());
            telemetry.addData("Distance", robot.distanceSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("Pictograph", pictograph);
            telemetry.addData("IMU Angle", robot.angles.firstAngle);
            telemetry.addData("Touch", robot.digitalTouch.getState());
            telemetry.update();

            imuTurnLeft(robot, 90, 0.5);
            Thread.sleep(1000);
            imuTurnRight(robot, 90, 0.5);
            Thread.sleep(1000);*/

            encodersStrafe(robot, 30, 0.25, "left");
            Thread.sleep(1000);
            encodersStrafe(robot, 30, 0.25, "right");
            Thread.sleep(1000);

            //touchSensorCount(robot, 2, 0.2);
            //distanceSensorCount(robot, 3, 0.3);
        //}
    }
}