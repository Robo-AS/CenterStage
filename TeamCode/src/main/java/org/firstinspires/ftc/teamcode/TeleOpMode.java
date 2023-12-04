package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.Arrays;
import java.util.List;

@TeleOp(name = "TeleOpMode", group = "Linear OpMode")
public class TeleOpMode extends MovementFunctions {
    List<Double> listOfArmPositions = Arrays.asList(0.0, 400.0, 600.0, 800.0);

    @Override
    public void runOpMode() {
        initialiseMecanum();
        initialiseArm();

        waitForStart();

        while (opModeIsActive()) {
            teleOpDrive();

            if(gamepad1.dpad_up && linearSlideMotor.getCurrentPosition()!= listOfArmPositions.get(3)){
                double current_position = linearSlideMotor.getCurrentPosition();
                int target_position_index = listOfArmPositions.indexOf(current_position) + 1;
                double target_position = listOfArmPositions.get(target_position_index);

                armMovement(1, target_position);
            }

            if(gamepad1.dpad_down){
                //...
            }

        }
    }
}
