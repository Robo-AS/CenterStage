package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
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
        servoPlane.setPosition(0.25); //75 grade rotatie
    }


}
