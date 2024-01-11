package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Claw {

    private Servo clawRight;
    private Servo clawLeft;

    private double clawOpen = 0.75;
    private double clawClosed = 0;

    private boolean rightOpen = false;
    private boolean leftOpen = false;

//    private TriggerReader leftTrigger;
//    private TriggerReader rightTrigger;

    public Claw(HardwareMap hardwareMap){
        clawRight = hardwareMap.get(Servo.class, "clawRight");
        clawLeft = hardwareMap.get(Servo.class, "clawLeft");
        
        clawRight.setDirection(Servo.Direction.REVERSE);

        clawLeft.setPosition(0);
        clawRight.setPosition(0);

        rightOpen = false;
        leftOpen = false;

    }




    public void teleop(GamepadEx gamepad, Telemetry telemetry){

//        leftTrigger.readValue();
//        rightTrigger.readValue();

        if(gamepad.wasJustPressed(GamepadKeys.Button.A)){
            leftOpen = !leftOpen;
            clawLeft.setPosition(leftOpen ? clawOpen : clawClosed);
        }
        if(gamepad.wasJustPressed(GamepadKeys.Button.B)){
            rightOpen = !rightOpen;
            clawRight.setPosition(rightOpen ? clawOpen : clawClosed);
        }

        telemetry.addData("clawLeft", clawLeft.getPosition());
        telemetry.addData("clawRight", clawRight.getPosition());

    }

}