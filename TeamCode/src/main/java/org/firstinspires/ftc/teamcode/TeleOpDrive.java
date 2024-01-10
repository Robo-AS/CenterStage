package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Mecanum;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.arcrobotics.ftclib.gamepad.GamepadEx;


@TeleOp(name = "TeleOpDrive", group= "Linear Opmode")
public class TeleOpDrive extends LinearOpMode {

    GamepadEx driver;
    GamepadEx operator;

    Mecanum drive;
    //Claw claw;

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);

        drive = new Mecanum(hardwareMap);
        //claw = new Claw(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            driver.readButtons();
            operator.readButtons();

            drive.teleop(driver,telemetry);
            //claw.teleop(operator,telemetry);

            if(driver.wasJustPressed(GamepadKeys.Button.A))
            {
                telemetry.addLine("sus");
            }


            telemetry.update();
        }

    }

}