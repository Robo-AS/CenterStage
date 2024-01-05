package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.ArrayList;
import java.util.List;

public class MovementFunctions extends LinearOpMode {
    public DcMotor frontRight, frontLeft, backRight, backLeft;
    public DcMotor linearSlideMotor, circularMovementMotor;
    public Servo servoLeftClaw;
    public Servo servoRightClaw;
    public Servo servoClawAngle;

    VisionPortal visionPortal;
    AprilTagProcessor aprilTag;
    AprilTagProcessor.Builder aprilTagBuilder;
    public IMU imu;

    public List<AprilTagDetection> aprilTagDetections;

    static final double COUNTS_PER_MOTOR_REV_CIRCULAR = 145.1; // motor 1150 rpm
    static final double COUNTS_PER_MOTOR_REV_LINEAR = 384.5; // motor 435 rpm
    static final double DRIVE_GEAR_REDUCTION_LINEAR = 1;
    static final double DRIVE_GEAR_REDUCTION_CIRCULAR = 28;
    public static double PULLEY_CIRCUMFERENCE_MM = 35.65 * Math.PI; // aprox. 122 mm
    static final double COUNTS_PER_PULLEY_REV = COUNTS_PER_MOTOR_REV_LINEAR * DRIVE_GEAR_REDUCTION_LINEAR; // 751.8
                                                                                                           // ticks
    static final double COUNTS_PER_MM = COUNTS_PER_PULLEY_REV / PULLEY_CIRCUMFERENCE_MM; // aprox 6.162 ticks/mm

    static final double COUNTS_PER_GEAR_REV = COUNTS_PER_MOTOR_REV_CIRCULAR * DRIVE_GEAR_REDUCTION_CIRCULAR; // 4062.8
                                                                                                             // ticks
    public static double COUNTS_PER_DEGREE = COUNTS_PER_GEAR_REV / 360; // aprox 11.285 tiks/degree

    // 145.1 ticks - o revolutie a motorului
    // 145.1 * 28 - 28 de revolutii ale motorului
    // - 28 de revolutii - revolutie worm gear
    // - revolutie worm gear - 360 de grade brat
    // 145.1 * 28 - 360 de grade brat
    // 11.285 - 1 grad brat

    public void initAprilTag() {

        // Create the AprilTag processor by using a builder.

        aprilTagBuilder = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true);

        aprilTag = aprilTagBuilder.build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 .. Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 .. Detect 2" Tag from 6 feet away at 22 Frames per second
        // Decimation = 3 .. Detect 2" Tag from 4 feet away at 30 Frames Per Second
        // Decimation = 3 .. Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);

        // Create the vision portal by using a builder.
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam"))
                .addProcessor(aprilTag)
                .build();
    }

    public List<AprilTagDetection> getDetections() {
        aprilTagDetections = aprilTag.getDetections();
        return aprilTagDetections;
    }

    public void TestMecanum() {
        MotorValues motorValues = new MotorValues(0.5);
        applyMotorValues(motorValues);
    }

    public void initialiseMecanum() {

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        switchMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;

        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientation));
    }

    public void initialiseArm() {
        linearSlideMotor = hardwareMap.get(DcMotor.class, "linearSlideMotor");
        circularMovementMotor = hardwareMap.get(DcMotor.class, "circularMovementMotor");

        circularMovementMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        circularMovementMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        circularMovementMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        linearSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        servoLeftClaw = hardwareMap.get(Servo.class, "servoLeftClaw");
        servoRightClaw = hardwareMap.get(Servo.class, "servoRightClaw");
        servoClawAngle = hardwareMap.get(Servo.class, "servoClawAngle");

        servoRightClaw.setDirection(Servo.Direction.REVERSE);

        servoLeftClaw.setPosition(0);
        servoRightClaw.setPosition(0);
        servoClawAngle.setPosition(0);

    }

    public void setClawLeft(boolean open) {
        if (open)
            servoLeftClaw.setPosition(0);
        else
            servoLeftClaw.setPosition(0.75);
    }

    public void setClawRight(boolean open) {
        if (open)
            servoRightClaw.setPosition(0);
        else
            servoRightClaw.setPosition(0.75);
    }

    public void teleOpDrive() {

        switchMotorModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.1;
        double rx = gamepad1.right_stick_x;
        // telemetry.addData("y",y);
        // telemetry.addData("x",x);
        // telemetry.addData("rx",rx);

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y - x - rx) / denominator;
        double frontRightPower = (y + x + rx) / denominator;
        double backLeftPower = (y + x - rx) / denominator;
        double backRightPower = (y - x + rx) / denominator;
        //
        // telemetry.addData("fl", frontLeftPower);
        // telemetry.addData("fr", frontRightPower);
        // telemetry.addData("bl", backLeftPower);
        // telemetry.addData("br", backRightPower);

        MotorValues motorValues = new MotorValues(frontLeftPower, frontRightPower, backLeftPower, backRightPower);

        if (gamepad1.left_bumper)
            motorValues.slowMode();

        motorValues.normaliseValues();
        applyMotorValues(motorValues);

    }

    public void teleOpDriveRelative() {
        switchMotorModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double x = gamepad1.left_stick_x * 1.1;
        double y = -gamepad1.left_stick_y;
        double rx = gamepad1.right_stick_x;

        double angle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        double newX = x * Math.cos(angle) - y * Math.sin(angle);
        double newY = x * Math.sin(angle) + y * Math.cos(angle);

        x = newX;
        y = newY;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double frontRightPower = (-y + x + rx) / denominator;
        double backLeftPower = (-y + x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        MotorValues motorValues = new MotorValues(frontLeftPower, frontRightPower, backLeftPower, backRightPower);

        if (gamepad1.left_bumper)
            motorValues.slowMode();

        motorValues.normaliseValues();
        applyMotorValues(motorValues);

    }

    public void switchMotorModes(DcMotor.RunMode x) {
        frontLeft.setMode(x);
        frontRight.setMode(x);
        backLeft.setMode(x);
        backRight.setMode(x);

        while ((frontLeft.getMode() != x || frontRight.getMode() != x || backLeft.getMode() != x
                || backRight.getMode() != x) && opModeIsActive())
            ;
    }

    public void applyMotorValues(MotorValues motorValues) {
        frontLeft.setPower(motorValues.fl);
        frontRight.setPower(motorValues.fr);
        backLeft.setPower(motorValues.bl);
        backRight.setPower(motorValues.br);
    }

    public void armLinearMovement(double power, double linear_slide_mm) {
        int targetTicks = (int) (linear_slide_mm * COUNTS_PER_MM);
        linearSlideMotor.setTargetPosition(targetTicks);
        linearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlideMotor.setPower(power);
    }

    public void armCircularMovement(double power, double degrees) {
        int targetTicks = (int) (degrees * COUNTS_PER_DEGREE);
        circularMovementMotor.setTargetPosition(targetTicks);
        circularMovementMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        circularMovementMotor.setPower(power);
    }

    @Override
    public void runOpMode() throws InterruptedException {

    }
}
