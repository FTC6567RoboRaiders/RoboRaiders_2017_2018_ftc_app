package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * Created by Katelin Zichittella on 9/24/2016.
 */

@Autonomous
@Disabled

public class SampleAutonomous extends SampleAutonomousHeader {

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();

        waitForStart();

        move(1.0, 1.0);
        Thread.sleep(1000);
        move(0.0, 0.0);
        Thread.sleep(500);

        move(-1.0, -1.0);
        Thread.sleep(1000);
        move(0.0, 0.0);
        Thread.sleep(500);

        move(1.0, -1.0);
        Thread.sleep(1000);
        move(0.0, 0.0);
        Thread.sleep(500);

        move(-1.0, 1.0);
        Thread.sleep(1000);
        move(0.0, 0.0);
        Thread.sleep(500);
    }
}