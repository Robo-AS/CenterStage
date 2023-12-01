package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOpMode", group = "Linear OpMode")
public class TeleOpMode extends MovementFunctions {

    @Override
    public void runOpMode() {
        initialiseMecanum();
        initialiseArm();

        waitForStart();

        while (opModeIsActive()) {
            teleOpDrive();

        }
    }
}
