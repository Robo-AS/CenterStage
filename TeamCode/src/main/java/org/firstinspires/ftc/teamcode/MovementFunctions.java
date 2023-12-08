package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.ArrayList;

public class MovementFunctions extends LinearOpMode {
    public DcMotor frontRight, frontLeft, backRight, backLeft;
    public DcMotor linearSlideMotor, circularMovementMotor;
    public Servo servoLeftClaw;
    public Servo servoRightClaw;
    public Servo servoClawAngle;



    public IMU imu;


    //HashMap<String, Double> ArmValues = new HashMap<>();

    public static double ticks_rev_223 = 751.8;
    public static double gear_ratio = 1;                  //the gear ration may be wrong, check the used gears
    public static double counts_per_gear_rev = ticks_rev_223 * gear_ratio;
    public static double counts_per_degree = counts_per_gear_rev/360;
    public static double diameter_mm_cable_pulley = 42.5;//trebuie masurat diametrul mosorului

    ArrayList<Boolean> booleanArray = new ArrayList<Boolean>();
    int booleanIncrementer = 0;

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

        circularMovementMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        circularMovementMotor.setTargetPosition(0);
        circularMovementMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        circularMovementMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        servoLeftClaw = hardwareMap.get(Servo.class, "servoLeftClaw");
        servoRightClaw = hardwareMap.get(Servo.class, "servoRightClaw");
        servoClawAngle = hardwareMap.get(Servo.class, "servoClawAngle");

        servoLeftClaw.setPosition(0);
        servoRightClaw.setPosition(0);
        servoClawAngle.setPosition(0);

        servoRightClaw.setDirection(Servo.Direction.REVERSE);
    }

    public void setClawLeft(boolean open){
        if(open)
            servoLeftClaw.setPosition(0);
        else
            servoLeftClaw.setPosition(0.75);
    }

    public void setClawRight(boolean open){
        if(open)
            servoRightClaw.setPosition(0);
        else
            servoRightClaw.setPosition(0.75);
    }

    public void teleOpDrive() {

        switchMotorModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x*1.1;
        double rx = gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double frontRightPower = ( -y + x + rx) / denominator;
        double backLeftPower = ( -y + x - rx) / denominator;
        double backRightPower = ( y + x - rx) / denominator;

        MotorValues motorValues= new MotorValues(frontLeftPower, frontRightPower, backLeftPower, backRightPower);

        if (gamepad1.left_bumper)
            motorValues.slowMode();

        motorValues.normaliseValues();
        applyMotorValues(motorValues);

    }

    public void teleOpDriveRelative() {
        switchMotorModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double x = gamepad1.left_stick_x*1.1;
        double y = -gamepad1.left_stick_y;
        double rx = gamepad1.right_stick_x;

        double angle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        double newX = x * Math.cos(angle) - y * Math.sin(angle);
        double newY = x * Math.sin(angle) + y * Math.cos(angle);

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double frontRightPower = ( -y + x + rx) / denominator;
        double backLeftPower = ( -y + x - rx) / denominator;
        double backRightPower = ( y + x - rx) / denominator;

        MotorValues motorValues= new MotorValues(frontLeftPower, frontRightPower, backLeftPower, backRightPower);

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


    public int mm_to_ticks(double mm, double ticks_revolution, double diameter, double gear_ratio) {
        return (int) (((ticks_revolution * mm) / (diameter * Math.PI)) * gear_ratio);
    }



    public void armLinearMovement(double power, double position){

        int targetTicks = mm_to_ticks(position, ticks_rev_223, diameter_mm_cable_pulley, gear_ratio);
        linearSlideMotor.setTargetPosition(targetTicks);
        linearSlideMotor.setPower(power);
        linearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void armCircularMovement(double power, double degrees){
        int armPosition = (int)degrees;
        circularMovementMotor.setTargetPosition(armPosition);
        circularMovementMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        circularMovementMotor.setPower(power);
    }




    @Override
    public void runOpMode() throws InterruptedException {

    }
}
