package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.checkerframework.checker.units.qual.A;

import java.util.Arrays;
import java.util.List;

@TeleOp(name = "TeleOpMode", group = "Linear OpMode")
public class TeleOpMode extends MovementFunctions {
    List<Double> listOfLinearSlidePositions = Arrays.asList(0.0, 50.0, 100.0, 150.0);
    List<Double> listOfArmAngles = Arrays.asList(0.0, 30.0, 45.0, 60.0);


    private boolean dpadUpPreviousState = false;
    private boolean dpadDownPreviousState = false;

    private int arm_position_index = 0;

    private boolean openLeftClaw = false;
    private boolean openRightClaw = false;

    enum modes{
        MOVE,
        PLACE
    }


    modes mode = modes.MOVE;
    private double servos_initial_position =0.0;


    Toggler switchModes=new Toggler();
    Toggler openLeft= new Toggler();
    Toggler openRight = new Toggler();

    Toggler gamepadUp = new Toggler();
    Toggler gamepadDown = new Toggler();


    @Override
    public void runOpMode() {
        initialiseMecanum();
        initialiseArm();

        waitForStart();

        while (opModeIsActive()) {
            if(mode==modes.MOVE){
                telemetry.addLine("Move Mode");
                teleOpDriveRelative();

                if(openLeft.status==Toggler.STATUS.JUST_PRESSED){
                    openLeftClaw=!openLeftClaw;
                }
                if(openRight.status==Toggler.STATUS.JUST_PRESSED){
                    openRightClaw=!openRightClaw;
                }



            }else if(mode==modes.PLACE){


                if(gamepadUp.status == Toggler.STATUS.JUST_PRESSED){
                    arm_position_index=Math.min(3, arm_position_index+1);
                }

                if(gamepadDown.status == Toggler.STATUS.JUST_PRESSED){
                    arm_position_index=Math.max(0, arm_position_index-1);
                }

                double targetLinearSlidePosition = listOfLinearSlidePositions.get(arm_position_index);
                double targetAngle = listOfArmAngles.get(arm_position_index);

                //armLinearMovement(0.2, targetLinearSlidePosition);
                armCircularMovement(-0.2, targetAngle);


                telemetry.addData("arm pos target", circularMovementMotor.getTargetPosition());

                telemetry.addData("arm pos", circularMovementMotor.getCurrentPosition());
            }


            openLeft.update(gamepad2.x);
            openRight.update(gamepad2.b);
            switchModes.update(gamepad1.x);
            gamepadUp.update(gamepad1.dpad_up);
            gamepadDown.update(gamepad1.dpad_down);


            if(switchModes.status == Toggler.STATUS.JUST_PRESSED){
                if(mode==modes.MOVE){
                    mode = modes.PLACE;
                }else{
                    mode=modes.MOVE;
                }
            }

            telemetry.update();
        }
    }
}
