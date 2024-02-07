package org.firstinspires.ftc.teamcode.programs.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;

import org.firstinspires.ftc.robotcore.external.Telemetry;
@Config
public class Claw {

    private Servo clawRight;
    private Servo clawLeft;

    public static double clawOpen = 0.13;
    public static double clawClosed = 0;

    private boolean rightOpen = false;
    private boolean leftOpen = false;


    public Claw(HardwareMap hardwareMap){
        clawRight = hardwareMap.get(Servo.class, "clawRight");
        clawLeft = hardwareMap.get(Servo.class, "clawLeft");

        clawRight.setDirection(Servo.Direction.REVERSE);


        clawLeft.setPosition(clawClosed);
        clawRight.setPosition(clawClosed);

    }




    public void teleop(GamepadEx gamepad, Telemetry telemetry){


        if(gamepad.wasJustPressed(GamepadKeys.Button.B)){
            leftOpen = !leftOpen;
            clawLeft.setPosition(leftOpen ? clawOpen : clawClosed);
        }

        if(gamepad.wasJustPressed(GamepadKeys.Button.Y)){
            rightOpen = !rightOpen;
            clawRight.setPosition(rightOpen ? clawOpen : clawClosed);
        }

        telemetry.addData("openLeftClaw", leftOpen);
        telemetry.addData("openRightClaw", rightOpen);


    }


}