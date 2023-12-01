package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Arm_movement", group="Linear OpMode")
public class Arm_movement extends LinearOpMode {
    DcMotor linearSlideMotor, circularMovementMotor;

    int targetPosition = 0;
    double power = 0.75;


    @Override
    public void runOpMode(){
        linearSlideMotor = hardwareMap.get(DcMotor.class, "linearSlideMotor");
        circularMovementMotor = hardwareMap.get(DcMotor.class, "circularMovementMotor");


        linearSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        circularMovementMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        circularMovementMotor.setTargetPosition(0);
        circularMovementMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        circularMovementMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData(">", "Press Start to execute code");
        telemetry.update();

        waitForStart(); 

        while(opModeIsActive()){

            //Code for the circular movement
            if(Math.abs(gamepad1.left_stick_y) > 0.1 || Math.abs(gamepad1.left_stick_y) < -0.1){
                targetPosition += (int)(1.5 * gamepad1.left_stick_y);
            }
            else{
                targetPosition = circularMovementMotor.getTargetPosition();
            }

            circularMovementMotor.setTargetPosition(targetPosition);
            circularMovementMotor.setPower(power);

            //Code for linear slide movement
            if(gamepad1.y){
                linearSlideMotor.setPower(1);
            }

            if(gamepad1.a){
                linearSlideMotor.setPower(-1);
            }

            linearSlideMotor.setPower(0);

            telemetry.addData("Arm Position", circularMovementMotor.getCurrentPosition());
            telemetry.update();

        }
    }




}
