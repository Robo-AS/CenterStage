package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Climber;
import org.firstinspires.ftc.teamcode.subsystems.Mecanum;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;

import com.arcrobotics.ftclib.gamepad.GamepadEx;


@TeleOp(name = "TeleOpDrive", group= "Linear Opmode")
public class TeleOpDrive extends LinearOpMode {

    GamepadEx driver;
    GamepadEx operator;

    Mecanum drive;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {


        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        drive = new Mecanum(hardwareMap);
        claw = new Claw(hardwareMap);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            driver.readButtons();
            operator.readButtons();

            drive.teleop(driver,telemetry);
            claw.teleop(operator,telemetry);
            

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }

    }

}