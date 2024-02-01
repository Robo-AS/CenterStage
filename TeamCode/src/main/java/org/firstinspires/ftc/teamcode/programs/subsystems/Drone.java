package org.firstinspires.ftc.teamcode.programs.subsystems;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Drone {

    Servo servoPlane;

    public Drone(HardwareMap hardwareMap){
        servoPlane = hardwareMap.get(Servo.class, "servoPlane");
        servoPlane.setPosition(0);
    }

    public void teleop(GamepadEx gamepad, Telemetry telemetry){
        if(gamepad.wasJustPressed(GamepadKeys.Button.X))
            servoPlane.setPosition(0.2);
    }


}
