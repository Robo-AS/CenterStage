package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOpMode", group = "Linear OpMode")
public class TeleOpMode extends Useful_Funtions {

    @Override
    public void runOpMode(){
        InitialiseMecanum();
        InitialiseArm();

        waitForStart();

        while(opModeIsActive()) {
            TeleOpDrive();

        }
    }
}
