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

    List<Double> listOfClawAngles = Arrays.asList(0.0, 30.0, 45.0, 60.0);

    private int arm_position_index = 0;

    private boolean openLeftClaw = false;
    private boolean openRightClaw = false;

    enum modes{
        MOVE,
        ENTER_PLACE,
        PLACE,

        AVION,
    }


    modes mode = modes.MOVE;

    Toggler switchModes=new Toggler();
    Toggler openLeft= new Toggler();
    Toggler openRight = new Toggler();

    Toggler gamepadUp = new Toggler();
    Toggler gamepadDown = new Toggler();


    @Override
    public void runOpMode() {
        initialiseMecanum();
        initialiseArm();
        initAprilTag();

        waitForStart();

        while (opModeIsActive()) {
            getDetections();
            if(mode==modes.MOVE){
                telemetry.addLine("Move Mode");
                teleOpDriveRelative();

                if(openLeft.status==Toggler.STATUS.JUST_PRESSED){
                    openLeftClaw=!openLeftClaw;
                }
                if(openRight.status==Toggler.STATUS.JUST_PRESSED){
                    openRightClaw=!openRightClaw;
                }

                if(openRightClaw){
                    servoRightClaw.setPosition(0);
                }else{
                    servoRightClaw.setPosition(0.073);
                }
                if(openLeftClaw){
                    servoLeftClaw.setPosition(0);
                }else{
                    servoLeftClaw.setPosition(0.073);
                }



            }else if(mode==modes.PLACE){


                if(gamepadUp.status == Toggler.STATUS.JUST_PRESSED){
                    arm_position_index=Math.min(3, arm_position_index+1);
                }

                if(gamepadDown.status == Toggler.STATUS.JUST_PRESSED){
                    arm_position_index=Math.max(0, arm_position_index-1);
                }

                armLinearMovement(0.2, listOfLinearSlidePositions.get(arm_position_index));
                armCircularMovement( 0.2, listOfArmAngles.get(arm_position_index));
                servoClawAngle.setPosition(listOfClawAngles.get(arm_position_index));
            }

            openLeft.update(gamepad2.x);
            openRight.update(gamepad2.b);
            switchModes.update(gamepad1.x);
            gamepadUp.update(gamepad1.dpad_up);
            gamepadDown.update(gamepad1.dpad_down);


            if(switchModes.status == Toggler.STATUS.JUST_PRESSED){
                if(mode==modes.MOVE){
                    mode = modes.ENTER_PLACE;
                }else if(mode==modes.ENTER_PLACE){
                    mode=modes.PLACE;
                }else {
                    mode=modes.MOVE;
                }
            }

            telemetry.addData("arm pos target", circularMovementMotor.getTargetPosition());
            telemetry.addData("arm pos", circularMovementMotor.getCurrentPosition());
            telemetry.addData("servo_angle", servoClawAngle.getPosition());
            telemetry.addData("servo_right", servoRightClaw.getPosition());
            telemetry.addData("servo_left", servoLeftClaw.getPosition());

            telemetry.addData("stuff", openLeftClaw);

            telemetry.addData("stuff", openRightClaw);

            telemetry.update();
        }
    }
}
