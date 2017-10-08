package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.roboraiders.Robot.Robot;


/**
 * Created by (mostly) Nick Urbin and (a little bit) Kevin McCrudden on 10/1/17.
 */

  class JewelAutonomous extends LinearOpMode {

    public Robot robot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {

        robot.initialize(hardwareMap);
        robot.servoJewel.setPosition(0.0);

        waitForStart();

        robot.colorSensor.red();
        robot.colorSensor.blue();

        telemetry.addData("Red", robot.colorSensor.red());
        telemetry.addData("Blue", robot.colorSensor.blue());
        telemetry.update();
// does the robot need to move forward at all? or no? discuss with program team. this programs assumes no.
        robot.servoJewel.setPosition(0.5);// lower arm with color sensor

        //assuming color sensor is mounted facing right

        //assuming red alliance

        // if (allianceColorRed == true){
        if (robot.colorSensor.red() > 675 &&  robot.colorSensor.red() <= 775) { // ball on the right is red
            //using motor power until we get math figured out for encoders

            robot.setDriveMotorPower(-1,1,1,-1); //strafe left
            Thread.sleep (500);
            robot.setDriveMotorPower(0,0,0,0);
            Thread.sleep (500);
            robot.setDriveMotorPower(1, -1, -1, 1); //strafe right to original position
            Thread.sleep (500);
            }
        else { //if the ball on the right is blue
            robot.setDriveMotorPower(1, -1, -1, 1); //strafe right
            Thread.sleep (500);
            robot.setDriveMotorPower(0,0,0,0);
            Thread.sleep (500);
            robot.setDriveMotorPower(-1,1,1,-1); //strafe left to original position
            Thread.sleep (500);

        }
        //}

        //assuming blue alliance

        //if {allianceColorRed == false){               // therefore blue alliance
        if (robot.colorSensor.blue() <= 675  && robot.colorSensor.blue() >= 575) {      // ball on the right is blue
            robot.setDriveMotorPower(-1, 1, 1, -1); //strafe left
            Thread.sleep(500);
            robot.setDriveMotorPower(0, 0, 0, 0);
            Thread.sleep(500);
            robot.setDriveMotorPower(1, -1, -1, 1); //strafe right to original position
            Thread.sleep(500);
            }
        else {
            robot.setDriveMotorPower(1, -1, -1, 1); //strafe right
            Thread.sleep (500);
            robot.setDriveMotorPower(0,0,0,0);
            Thread.sleep (500);
            robot.setDriveMotorPower(-1,1,1,-1); //strafe left to original position
            Thread.sleep (500);
        }




        //}
    }
}
