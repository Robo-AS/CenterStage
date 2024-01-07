package org.firstinspires.ftc.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Claw {

    private Servo clawRight;
    private Servo clawLeft;

    private double clawOpen = 0.75;
    private double clawClosed = 0;

    private boolean rightOpen = false;
    private boolean leftOpen = false;

    public Claw(HardwareMap hardwareMap){
        clawRight = hardwareMap.get(Servo.class, "clawRight");
        clawLeft = hardwareMap.get(Servo.class, "clawLeft");
        
        clawRight.setDirection(Servo.Direction.REVERSE);

        clawLeft.setPosition(0);
        clawRight.setPosition(0);
    }




    public void teleop(GamepadEx gamepad, Telemetry telemetry){
        
        if(gamepad.wasJustPressed(GamepadKeys.Trigger.LEFT_TRIGGER)){
            boolean leftOpen = !leftOpen;
            clawLeft.setPosition(leftOpen ? clawOpen : clawClosed);
        }
        if(gamepad.wasJustPressed(GamepadKeys.Trigger.RIGHT_TRIGGER)){
            boolean rightOpen = !rightOpen;
            clawRight.setPosition(rightOpen ? clawOpen : clawClosed);
        }

        telemetry.addData("clawLeft", clawLeft.getPosition());
        telemetry.addData("clawRight", clawRight.getPosition());

    }

}