package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.checkerframework.checker.units.qual.A;

import java.util.Arrays;
import java.util.List;

@TeleOp(name = "TeleOpMode", group = "Linear OpMode")
public class TeleOpMode extends MovementFunctions {

    enum modes{
        MOVE,
        PLACE
    }

    modes mode = modes.MOVE;

    @Override
    public void runOpMode() {
        initialiseMecanum();
        initialiseArm();

        waitForStart();

        while (opModeIsActive()) {

            if(mode==modes.MOVE){
                teleOpDrive();

            }else if(mode==modes.PLACE){

            }

            if(gamepad1.x){
                mode = mode==modes.MOVE ? modes.PLACE : modes.MOVE;
            }
        }
    }
}
