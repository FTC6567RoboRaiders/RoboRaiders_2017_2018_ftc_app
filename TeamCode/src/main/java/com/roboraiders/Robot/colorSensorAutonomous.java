package com.roboraiders.Robot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.roboraiders.reference.RevColorSensorNew;

/**
 * Created by Alex on 9/27/17.
 */

@Autonomous

public class colorSensorAutonomous extends RevColorSensorNew {

    ColorSensor color_sensor;

    public void init() {
        color_sensor = hardwareMap.colorSensor.get("Color");
    }

    @Override
    public void loop() {

        color_sensor.red();
        color_sensor.blue();

        telemetry.addData("Color", color_sensor.red());
        telemetry.addData("Color", color_sensor.blue());
        telemetry.update();


    }
}