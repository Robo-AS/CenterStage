package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Useful_Funtions extends LinearOpMode {
    public DcMotor frontRight, frontLeft, backRight, backLeft;
    public DcMotor linearSlideMotor, circularMovementMotor;
    public void InitialiseMecanum(){

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        SwitchMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }


    public void InitialiseArm(){
        linearSlideMotor = hardwareMap.get(DcMotor.class, "linearSlideMotor");
        circularMovementMotor = hardwareMap.get(DcMotor.class, "circularMovementMotor");


        linearSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        circularMovementMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        circularMovementMotor.setTargetPosition(0);
        circularMovementMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        circularMovementMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void TeleOpDrive(){
        SwitchMotorModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double x = 0;
        double y = 0;
        double rotation = 0;


         x = gamepad1.left_stick_x;
         y = gamepad1.left_stick_y;
         rotation = gamepad1.right_stick_x;

        boolean axis = Math.abs(x) > Math.abs(y);

        double direction = axis ? -1 : 1;

        double power_fl = direction * (axis ? x : y);
        double power_fr = axis ? x : y;
        double power_bl = axis ? x : y;
        double power_br = direction * (axis ? x : y);


        if(rotation != 0){
            if(rotation < 0){
                power_fr = rotation;
                power_br = rotation;
                power_fl = -rotation;
                power_bl = -rotation;
            } else{
                power_fr = rotation;
                power_br = rotation;
                power_fl = -rotation;
                power_bl = -rotation;
            }
        }

//        double power_fl = -x - y + rotation;
//        double power_fr = -x + y + rotation;
//        double power_bl = -x + y - rotation;
//        double power_br = -x - y - rotation;

        MotorValues motorValues = new MotorValues(power_fl, power_fr, power_bl, power_br);
        if (gamepad1.left_bumper) motorValues.SlowMode();

        motorValues.NormaliseValues();
        ApplyMotorValues(motorValues);

    }








    public void SwitchMotorModes(DcMotor.RunMode x) {
        frontLeft.setMode(x);
        frontRight.setMode(x);
        backLeft.setMode(x);
        backRight.setMode(x);

        while ((frontLeft.getMode() != x || frontRight.getMode() != x || backLeft.getMode() != x || backRight.getMode() != x) && opModeIsActive());
    }



    public void ApplyMotorValues(MotorValues motorValues) {
        frontLeft.setPower(motorValues.fl);
        frontRight.setPower(motorValues.fr);
        backLeft.setPower(motorValues.bl);
        backRight.setPower(motorValues.br);
    }


    @Override
    public void runOpMode() throws InterruptedException{

    }
}
