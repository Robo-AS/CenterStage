package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.Arrays;
import java.util.List;

@TeleOp(name = "TeleOpMode", group = "Linear OpMode")
public class TeleOpMode extends AutonomousFunctions {
    List<Double> listOfLinearSlidePositions = Arrays.asList(0.0, 50.0, 100.0, 150.0);
    List<Double> listOfArmAngles = Arrays.asList(0.0, 30.0, 45.0, 60.0);

    List<Double> listOfClawAngles = Arrays.asList(0.0, 30.0, 45.0, 60.0);

    private int arm_position_index = 0;

    private boolean openLeftClaw = false;
    private boolean openRightClaw = false;

    private int needid=2;

    enum MODES{
        MOVE,
        ENTER_PLACE,
        PLACE,

        ENTER_MOVE,
        AVION,
    }


    MODES mode = MODES.MOVE;

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

        //TestMecanum();


        waitForStart();


        while (opModeIsActive()) {
            getDetections();
            if(mode==MODES.MOVE){
                telemetry.addLine("Move Mode");

                teleOpDrive();

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
            }else if(mode==MODES.ENTER_PLACE){
                telemetry.addLine("Enter Place Mode");

                List<AprilTagDetection> detections = getDetections();
                AprilTagDetection good;

                boolean found=false;
                for(AprilTagDetection detection : detections){
                    if(detection.metadata.id==needid){
                        good = detection;
                        found=true;
                        break;
                    }
                }

                if(found==false){
                    telemetry.addLine("NO APRIL TAG YOU DUMB FUCK");
                    mode=MODES.MOVE;
                    break;
                }

                //alignToActualDetection(good);


                mode=MODES.PLACE;
            }
            else if(mode==MODES.PLACE){


                telemetry.addLine("Place Mode");
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
            else if(mode==MODES.ENTER_MOVE){
                //aici reseteaza chestii cand intram in move
                telemetry.addLine("Enter Move Mode");
                mode=MODES.MOVE;
            }


            openLeft.update(gamepad2.x);
            openRight.update(gamepad2.b);
            switchModes.update(gamepad1.x);
            gamepadUp.update(gamepad1.dpad_up);
            gamepadDown.update(gamepad1.dpad_down);


            if(switchModes.status == Toggler.STATUS.JUST_PRESSED){
                if(mode==MODES.MOVE){
                    mode = MODES.ENTER_PLACE;
                }if(mode==MODES.PLACE) {
                    mode=MODES.ENTER_MOVE;
                }
            }

            telemetry.addData("linearSlideMotor", linearSlideMotor.getTargetPosition());
            telemetry.addData("circularMovementMotor", circularMovementMotor.getCurrentPosition());
            telemetry.addData("ServoClawAngle", servoClawAngle.getPosition());
            //telemetry.addData("servo_right", servoRightClaw.getPosition());
            //telemetry.addData("servo_left", servoLeftClaw.getPosition());

            //telemetry.addData("openLeftClaw", openLeftClaw);

            //telemetry.addData("openRightClaw", openRightClaw);

            telemetry.addData("arm_position_index", arm_position_index);

            telemetry.update();
        }
    }
}
