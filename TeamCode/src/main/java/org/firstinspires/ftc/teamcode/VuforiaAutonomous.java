package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

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

        lastKnownLocation = createMatrix(0, 0, 0, 0, 0, 0);

        waitForStart();

        visionTargets.activate();

        while (opModeIsActive()) {

            OpenGLMatrix latestLocation = listener.getUpdatedRobotLocation();

            if (latestLocation != null) {

                lastKnownLocation = latestLocation;
            }

            float[] coordinates = lastKnownLocation.getTranslation().getData();

            robotX = coordinates[0];
            robotY = coordinates[1];
            robotAngle = Orientation.getOrientation(lastKnownLocation, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;

            telemetry.addData("Tracking " + target.getName(), listener.isVisible());
            telemetry.addData("Last Known Location", formatMatrix(lastKnownLocation));
            telemetry.addData("robotX", robotX);
            telemetry.addData("robotY", robotY);
            telemetry.addData("robotAngle", robotAngle);
            telemetry.update();


        }
    }
}