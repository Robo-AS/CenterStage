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

@TeleOp(name = "TeleOpDrive", group= "Linear Opmode")
public class TeleOpDrive extends LinearOpMode {

    Mecanum drive;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        drive = new Mecanum(hardwareMap);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            drive.teleop(gamepad1,telemetry);
            
            

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }

    }

}