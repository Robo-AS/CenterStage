package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class UsefulFunctions extends LinearOpMode {

    public DcMotor frontLeft, frontRight, backLeft, backRight;

    public void Initialise() {

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        setMotorModes(DcMotor.RunMode.RUN_USING_ENCODERS);


        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //telling the motors which direction to take(backwords or forward)
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    //
    public void setMotorModes(DcMotor.RunMode mode){

        frontLeft.setMode(mode);
        frontRight.setMode(mode);
        backLeft.setMode(mode);
        backRight.setMode(mode);
        
    }

    public void applyMotorValues(MotorValues mv){
        frontLeft.setPower(mv.fl);
        frontRight.setPower(mv.fr);
        backLeft.setPower(mv.bl);
        backRight.setPower(mv.br);
    }

    public void TeleOpDrive() {
        setMotorModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double frontRightPower = ( -y + x - rx) / denominator;
        double backLeftPower = ( -y + x + rx) / denominator;
        double backRightPower = ( y + x - rx) / denominator;

        MotorValues motorValues= new MotorValues(frontLeftPower, frontRightPower, backLeftPower, backRightPower);

        if (gamepad1.left_bumper) motorValues.SlowMode();

        telemetry.addData("FrontLeft", frontLeftPower);
        telemetry.addData("FrontRight", frontRightPower);
        telemetry.addData("BackLeft", backLeftPower);
        telemetry.addData("BackRight", backRightPower);

        telemetry.update();

        motorValues.NormaliseValues();
        applyMotorValues(motorValues);
    }
    //e gen sa fie clasa abstracta
    @Override
    public void runOpMode () throws InterruptedException {
    }
}
