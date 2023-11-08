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

        setDirection(DcMotorSimple.Direction.FORWARD);


    }

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




    public void setDirection(DcMotorSimple.Direction mode){

        frontLeft.setDirection(mode);
        frontRight.setDirection(mode);
        backLeft.setDirection(mode);
        backRight.setDirection(mode);
    }

    public void TeleOpDrive() {
        setMotorModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double x = gamepad1.left_stick_x;
        double y = gamepad1.left_stick_y;
        double rotation = gamepad1.right_stick_x;

        double power_fl = x + y - rotation;
        double power_fr = -x -y - rotation;
        double power_bl = -x + y - rotation;
        double power_br = x - y - rotation;

        MotorValues motorValues = new MotorValues(power_fl, power_fr, power_bl, power_br);

        if (gamepad1.left_bumper) motorValues.SlowMode();

        motorValues.NormaliseValues();
        applyMotorValues(motorValues);
    }
    //e gen sa fie clasa abstracta
    @Override
    public void runOpMode () throws InterruptedException {
    }
}
