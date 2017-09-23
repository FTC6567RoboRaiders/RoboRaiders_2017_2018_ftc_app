/*
 */

package com.roboraiders.Robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class will be used to define all functions for our 2017 - 2018 robot.
 *
 */
public class Robot
{
    /*Robot Motors */
    public DcMotor  motorleftFront   = null;
    public DcMotor  motorrightFront  = null;
    public DcMotor  motorleftBack     = null;
    public DcMotor  motorrightBack    = null;


    /* local OpMode members. */
    HardwareMap hwMap           =  null;

    /* Constructor */
    public Robot(){

    }

    /**  init - initialize the robot
     *
     *
     *
     * @param ahwMap - hardware map for the robot
     *
     */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        motorleftFront  = hwMap.get(DcMotor.class, "left_Front");
        motorrightFront = hwMap.get(DcMotor.class, "right_Front");
        motorleftBack = hwMap.get(DcMotor.class, "left_Back");
        motorrightBack = hwMap.get(DcMotor.class, "right_Back");

        // Defines the directions the motors will spin
        motorleftFront.setDirection(DcMotor.Direction.FORWARD);
        motorrightFront.setDirection(DcMotor.Direction.REVERSE);
        motorleftBack.setDirection(DcMotor.Direction.FORWARD);
        motorrightBack.setDirection(DcMotorSimple.Direction.REVERSE);


        // Set all motors to zero power
        motorrightFront.setPower(0);
        motorleftFront.setPower(0);
        motorrightBack.setPower(0);
        motorleftBack.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        motorleftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorrightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorleftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorrightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



    }

    /** setDriveMotorPower
     *
     * @param leftFront
     * @param rightFront
     * @param leftBack
     * @param rightBack
     */
    public void setDriveMotorPower (float leftFront, float rightFront, float leftBack, float rightBack){

        motorleftFront.setPower(leftFront);
        motorrightFront.setPower(rightFront);
        motorleftBack.setPower(leftBack);
        motorrightBack.setPower(rightBack);

    }




 }



