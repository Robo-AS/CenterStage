package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Claw {

    private Servo rightWheel;
    private Servo leftWheel;








    public Claw(HardwareMap hardwareMap){
        rightWheel = hardwareMap.get(Servo.class, "clawRight");
        leftWheel = hardwareMap.get(Servo.class, "clawLeft");

    }




    public void teleop(GamepadEx gamepad, Telemetry telemetry){


        if(gamepad.wasJustPressed(GamepadKeys.Button.X)){
            rightWheel.setPosition(0);
            leftWheel.setPosition(1);
        }

        if(gamepad.wasJustPressed(GamepadKeys.Button.B)){
            rightWheel.setPosition(1);
            leftWheel.setPosition(0);
        }


        if(gamepad.wasJustPressed(GamepadKeys.Button.Y)){
            rightWheel.setPosition(0.5);
            leftWheel.setPosition(0.5);
        }




    }

}