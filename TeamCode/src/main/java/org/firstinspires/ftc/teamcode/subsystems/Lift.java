package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Lift {

    private DcMotor circularMovementMotor, linearSlideMotor;

    public Lift(HardwareMap hardwareMap){
        linearSlideMotor = hardwareMap.get(DcMotor.class, "linearSlideMotor");
        circularMovementMotor = hardwareMap.get(DcMotor.class, "circularMovementMotor");

        circularMovementMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        circularMovementMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        circularMovementMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        linearSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


    public void extindMax(GamepadEx gamepad, Telemetry telemetry, Arm arm){
        arm.armLinearMovement(0.1, 240); //nu mai stiu aici cat se extindea linear slide-ul max
//        while(linearSlideMotor.isBusy()){
//            telemetry.addData("Extending linear slide", linearSlideMotor.getCurrentPosition());
//        }
    }

    public void contract(GamepadEx gamepad, Telemetry telemetry, Arm arm){
        arm.armLinearMovement(0.1, 190);
    }

    public void liftTheThing(GamepadEx gamepad, Telemetry telemetry, Arm arm){
        arm.armCircularMovement(0.1, 60);
    }



}
