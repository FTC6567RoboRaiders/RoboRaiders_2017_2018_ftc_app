package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.roboraiders.Robot.Robot;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by Nick on 10/8/17.
 */

@Autonomous

public class SensorTest extends LinearOpMode {

    public Robot robot = new Robot();

    VuforiaLocalizer vuforia;

    @Override
    public void runOpMode() throws InterruptedException {

        initialize(hardwareMap);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AedUDNP/////AAAAGXH2ZpUID0KanSX9ZSR37LKFSFokxIqmy/g0BNepdA9EepixxnO00qygLnMJq3Fg9gZxnkUJaKgk14/UjhxPWVQIs90ZXJLc21NvQvOeZ3dOogagVP8yFnFQs2xCijGmC/CE30ojlAnbhAhqz1y4tZPW2QkK5Qt0xCakTTSAw3KPQX2mZxX+qMxI2ljrN0eaxaKVnKnAUl8x3naF1mez7f9c8Xdi1O5auL0ePdG6bJhWjEO1YwpSd8WkSzNDEkmw20zpQ7zaOOPw5MeUQUr9vAS0fef0GnLjlS1gb67ajUDlEcbbbIeSrLW/oyRGTil8ueQC2SWafdspSWL3SJNaQKWydies23BxJxM/FoLuYYjx";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");
        relicTrackables.activate();

        waitForStart();

        while (opModeIsActive()) {

            /*RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            robot.colorSensor.red();
            robot.colorSensor.blue();

            if (vuMark.equals(RelicRecoveryVuMark.LEFT)) {

                robot.pictograph = "LEFT";
            } else if (vuMark.equals(RelicRecoveryVuMark.CENTER)) {

                robot.pictograph = "CENTER";
            } else if (vuMark.equals(RelicRecoveryVuMark.RIGHT)) {

                robot.pictograph = "RIGHT";
            } else {

                robot.pictograph = "UNKNOWN";
            }

            telemetry.addData("Red", robot.colorSensor.red());
            telemetry.addData("Blue", robot.colorSensor.blue());
            telemetry.addData("Distance", robot.distanceSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("Pictograph", robot.pictograph);
            telemetry.addData("IMU Angle", robot.angles.firstAngle);
            telemetry.update();*/

            encodersStrafeRight(20, 0.5);
        }
    }

    public void initialize(HardwareMap ahwMap) {

        // Save reference to hardware map
        robot.hwMap = ahwMap;

        robot.colorSensor = robot.hwMap.get(ColorSensor.class, "sensor_color_distance");
        robot.distanceSensor = robot.hwMap.get(DistanceSensor.class, "sensor_color_distance");
        robot.imu = robot.hwMap.get(BNO055IMU.class, "imu");
        robot.parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        robot.imu.initialize(robot.parameters);
    }

    public void encodersStrafeRight(int distance, double power) {

        if (opModeIsActive()) {

            robot.motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            int DIAMETER = 4;
            int GEAR_RATIO = 1;
            int PULSES = 1120;
            double CIRCUMFERENCE = Math.PI * DIAMETER;
            double ROTATIONS = (distance / CIRCUMFERENCE) * GEAR_RATIO;
            double COUNTS = PULSES * ROTATIONS;

            COUNTS = COUNTS + Math.abs(robot.motorBackRight.getCurrentPosition());

            robot.setDriveMotorPower(power, -power, -power, power);

            while (robot.motorBackRight.getCurrentPosition() < COUNTS && opModeIsActive()) {

            }

            robot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0);
        }
    }
}