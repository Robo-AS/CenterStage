package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.checkerframework.checker.units.qual.A;

import java.util.Arrays;
import java.util.List;

@TeleOp(name = "TeleOpMode", group = "Linear OpMode")
public class TeleOpMode extends MovementFunctions {

    Gamepad lastgamepad1=new Gamepad();
    Gamepad lastgamepad2=new Gamepad();
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
                teleOpDriveRelative();

            }else if(mode==modes.PLACE){

            }

            if(gamepad1.x && lastgamepad1.x==false){
                mode = mode==modes.MOVE ? modes.PLACE : modes.MOVE;
            }
            lastgamepad1 = gamepad1;
            lastgamepad2 = gamepad2;
        }
    }
}
