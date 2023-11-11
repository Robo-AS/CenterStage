package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOpMode", group = "Linear OpMode")
public class TeleOpMode extends MecanumDrive {

    @Override
    public void runOpMode(){
        InitialiseMotors();
        waitForStart();
        while(opModeIsActive()) {
            MotorPower();
        }
    }
}
