package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.checkerframework.checker.units.qual.A;

import java.util.Arrays;
import java.util.List;

@TeleOp(name = "TeleOpMode", group = "Linear OpMode")
public class TeleOpMode extends MovementFunctions {

    int[] angles = {180,210,240,270};
    int[] distances = {0,0,0,0};
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
                telemetry.addLine("Move Mode");
                teleOpDriveRelative();

            }else if(mode==modes.PLACE){
                telemetry.addLine("Place Mode");

                boolean[] dpad = {gamepad1.dpad_up, gamepad1.dpad_right, gamepad1.dpad_down, gamepad1.dpad_left};

                for(int i=0;i<4;i++){
                    if(dpad[i]){
                        armCircularMovement(0.1, angles[i]);
                    }
                }


            }

            if(gamepad1.x && !lastgamepad1.x){
                if(mode==modes.MOVE){
                    mode = modes.PLACE;
                }else{
                    mode=modes.MOVE;
                }

            }
            telemetry.addData("x1",gamepad1.x);
            telemetry.addData("x2",lastgamepad1.x);


            lastgamepad2 = gamepad2;

            telemetry.update();
        }
    }
}
