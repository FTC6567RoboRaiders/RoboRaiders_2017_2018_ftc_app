package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

/**
 * Created by Katelin Zichittella on 7/27/2017.
 */

public abstract class AutonomousHeaderPractice extends LinearOpMode {

    DcMotor motorLeft, motorRight;

    byte[] sensorColorCache;
    I2cDevice sensorColor;
    I2cDeviceSynch sensorColorReader;

    OpticalDistanceSensor sensorODS;

    public void initialize() {

        motorLeft = hardwareMap.dcMotor.get("motorLeft");
        motorRight = hardwareMap.dcMotor.get("motorRight");

        sensorODS = hardwareMap.opticalDistanceSensor.get("sensorODS");

        motorRight.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Initialized", true);
        telemetry.update();
    }

    public void stopAtDistance(double distance, double power) { // Distance is 0 for farthest away
                                                                // and 1 for closest
        if (opModeIsActive()) {

            motorLeft.setPower(power);
            motorRight.setPower(power);

            while (sensorODS.getLightDetected() < distance && opModeIsActive()) {

                telemetry.addData("Normal", sensorODS.getLightDetected());
                telemetry.update();
            }

            motorLeft.setPower(0.0);
            motorRight.setPower(0.0);
        }
    }
}