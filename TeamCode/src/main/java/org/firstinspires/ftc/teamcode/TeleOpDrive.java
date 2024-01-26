package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Mecanum;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Drone;

import com.arcrobotics.ftclib.gamepad.GamepadEx;


@TeleOp(name = "TeleOpDrive", group= "Linear Opmode")
public class TeleOpDrive extends LinearOpMode {

    GamepadEx driver;
    GamepadEx operator;

    Mecanum drive;
    Claw claw;
    Arm arm;
    Drone drone;


    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        drive = new Mecanum(hardwareMap);
        claw = new Claw(hardwareMap);
        arm = new Arm(hardwareMap);
        drone = new Drone(hardwareMap);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            driver.readButtons();
            operator.readButtons();

            drive.teleop(driver,telemetry);
            claw.teleop(operator,telemetry);
            arm.teleop(operator, telemetry);
            drone.teleop(driver, telemetry);


            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }

    }

}