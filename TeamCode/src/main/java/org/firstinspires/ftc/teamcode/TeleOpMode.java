package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOpMode", group = "Linear OpMode")
public class TeleOpMode extends Useful_Funtions {

    @Override
    public void runOpMode(){
        InitialiseComponents();

        waitForStart();

        while(opModeIsActive()) {
            TeleOpDrive();

            if(gamepad1.y){
                linearSlideMotor.setPower(1);
            }
            else linearSlideMotor.setPower(0);


            if(gamepad1.a){
                linearSlideMotor.setPower(-1);
            }
            else linearSlideMotor.setPower(0);
        }
    }
}
