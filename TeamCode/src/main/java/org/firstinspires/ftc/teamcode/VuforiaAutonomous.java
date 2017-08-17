package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * Created by Katelin Zichittella on 7/7/2017.
 */

@Autonomous
@Disabled

public class VuforiaAutonomous extends VuforiaAutonomousHeader {

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();

        setupVuforia();

        waitForStart();

        visionTargets.activate();

        while (opModeIsActive()) {

            if (targetsAreVisible()) {
                cruiseControl(TARGET_DISTANCE);
            }

            moveRobot();

        }
    }
}