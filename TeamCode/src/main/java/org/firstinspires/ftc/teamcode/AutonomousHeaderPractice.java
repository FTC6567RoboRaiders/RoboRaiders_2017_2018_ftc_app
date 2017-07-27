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
        sensorColor = hardwareMap.i2cDevice.get("sensorColor");
        sensorODS = hardwareMap.opticalDistanceSensor.get("sensorODS");

        motorRight.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Initialized", true);
        telemetry.update();
    }

    public void shortDis (int distance) {

        motorLeft.setPower(1.0);
        motorRight.setPower(1.0);

        while (sensorODS.getLightDetected() < 100) {


        }

        motorLeft.setPower(0.0);
        motorRight.setPower(0.0);
    }
}
