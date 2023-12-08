package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@TeleOp(name = "TeleOpMode", group = "Linear OpMode")
public class TeleOpMode extends MovementFunctions {
    List<Double> listOfLinearSlidePositions = Arrays.asList(0.0, 50.0, 100.0, 150.0);
    List<Double> listOfArmAngles = Arrays.asList(0.0, 30.0, 45.0, 60.0);

    //ArrayList<Boolean> booleanArray = new ArrayList<Boolean>();
    //int booleanIncrementer = 0;
    private boolean dpadUpPreviousState = false;
    private boolean dpadDownPreviousState = false;

    private int arm_position_index = 0;

    private boolean openLeftClaw = false;
    private boolean openRightClaw = false;

    private double servos_initial_position =0.0;




    @Override
    public void runOpMode() {
        initialiseMecanum();
        initialiseArm();

        waitForStart();

        while (opModeIsActive()) {

            teleOpDriveRelative();

//            boolean dpad_up = gamepad1.dpad_up;
//            boolean dpad_down = gamepad1.dpad_down;

//            boolean dpad_up_pressed = ifPressed(dpad_up);
//            boolean dpad_down_pressed = ifPressed(dpad_down);


//            if(gamepad1.dpad_up  && arm_position_index!=3){
//                arm_position_index = arm_position_index +1;
//                double targetLinearSlidePosition = listOfLinearSlidePositions.get(arm_position_index);
//                double targetAngle = listOfArmAngles.get(arm_position_index);
//
//                //armLinearMovement(0.2, targetLinearSlidePosition);
//                armCircularMovement( 0.2, targetAngle);
//
//            }

//            if(gamepad1.dpad_down && arm_position_index!=0){
//                arm_position_index = arm_position_index -1;
//                double targetLinearSlidePosition = listOfLinearSlidePositions.get(arm_position_index);
//                double targetAngle = listOfArmAngles.get(arm_position_index);
//
//                //armLinearMovement(0.2, targetLinearSlidePosition);
//                armCircularMovement(-0.2, targetAngle);
//            }




            if (!gamepad1.dpad_up && dpadUpPreviousState) {
                dpadUpPreviousState = false;
                arm_position_index = arm_position_index -1;
                double targetLinearSlidePosition = listOfLinearSlidePositions.get(arm_position_index);
                double targetAngle = listOfArmAngles.get(arm_position_index);

                //armLinearMovement(0.2, targetLinearSlidePosition);
                armCircularMovement(-0.2, targetAngle);
            }
            if (gamepad1.dpad_up) dpadUpPreviousState = true;




            if(!gamepad1.dpad_down && dpadDownPreviousState){
                dpadDownPreviousState = false;
                arm_position_index = arm_position_index -1;
                double targetLinearSlidePosition = listOfLinearSlidePositions.get(arm_position_index);
                double targetAngle = listOfArmAngles.get(arm_position_index);

                //armLinearMovement(0.2, targetLinearSlidePosition);
                armCircularMovement(-0.2, targetAngle);
            }
            if(gamepad1.dpad_down) dpadDownPreviousState = true;


            telemetry.addData("position", arm_position_index);
            telemetry.addData("circularPosition", circularMovementMotor.getCurrentPosition());
            telemetry.addData("linearSlidePosition", linearSlideMotor.getCurrentPosition());
            telemetry.update();



            //Cleste deshis-inchis
            if(gamepad1.x && !openLeftClaw){
                servoLeftClaw.setPosition(0.073);//22 de grade din pozitia initiala
                openLeftClaw = true;
            }
            else if(gamepad1.x && openLeftClaw){
                servoLeftClaw.setPosition(servos_initial_position);
                openLeftClaw = false;
            }


            if (gamepad1.b && !openRightClaw) {
                servoRightClaw.setPosition(0.073);//22 de grade din pozitia initiala
                openRightClaw = true;
            }
            else if(gamepad1.b && openRightClaw){
                servoRightClaw.setPosition(servos_initial_position);
                openRightClaw = false;
            }


            // booleanIncrementer = 0;
        }






    }



//
//    private boolean ifPressed(boolean button){
//        boolean output = false;
//        if(booleanArray.size() == booleanIncrementer){
//            booleanArray.add(false);
//        }
//
//        boolean buttonWas = booleanArray.get(booleanIncrementer);
//
//        if(button != buttonWas && button){
//            output = true;
//        }
//
//        booleanArray.set(booleanIncrementer, button);
//        booleanIncrementer = booleanIncrementer + 1;
//        return  output;
//    }
}
