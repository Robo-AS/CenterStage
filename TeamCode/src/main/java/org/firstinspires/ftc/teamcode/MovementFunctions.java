package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class MovementFunctions extends LinearOpMode {
    public DcMotor frontRight, frontLeft, backRight, backLeft;
    public DcMotor linearSlideMotor, circularMovementMotor;

    public IMU imu;

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

        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;

        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientation));
    }

    public void initialiseArm() {
        linearSlideMotor = hardwareMap.get(DcMotor.class, "linearSlideMotor");
        circularMovementMotor = hardwareMap.get(DcMotor.class, "circularMovementMotor");

        linearSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        circularMovementMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        circularMovementMotor.setTargetPosition(0);
        circularMovementMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        circularMovementMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void teleOpDrive() {

        switchMotorModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double x = gamepad1.left_stick_x;
        double y = gamepad1.left_stick_y;
        double rotation = gamepad1.right_stick_x;

        double powerFrontLeft = x + y - rotation;
        double powerFrontRight = -x - y - rotation;
        double powerBackLeft = -x + y - rotation;
        double powerBackRight = x - y - rotation;

        MotorValues motorValues = new MotorValues(powerFrontLeft, powerFrontRight, powerBackLeft, powerBackRight);

        if (gamepad1.left_bumper)
            motorValues.slowMode();

        motorValues.normaliseValues();
        applyMotorValues(motorValues);

    }

    public void teleOpDriveRelative() {

        switchMotorModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double x = gamepad1.left_stick_x;
        double y = gamepad1.left_stick_y;
        double rotation = gamepad1.right_stick_x;

        double angle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        double newX = x * Math.cos(angle) - y * Math.sin(angle);
        double newY = x * Math.sin(angle) + y * Math.cos(angle);

        double powerFrontLeft = newX + newY - rotation;
        double powerFrontRight = -newX - newY - rotation;
        double powerBackLeft = -newX + newY - rotation;
        double powerBackRight = newX - newY - rotation;

        MotorValues motorValues = new MotorValues(powerFrontLeft, powerFrontRight, powerBackLeft, powerBackRight);

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

    @Override
    public void runOpMode() throws InterruptedException {

    }
}
