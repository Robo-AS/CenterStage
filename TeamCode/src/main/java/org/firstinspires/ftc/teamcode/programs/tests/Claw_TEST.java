package org.firstinspires.ftc.teamcode.programs.tests;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name = "Claw_TEST", group= "Linear Opmode")
public class Claw_TEST extends LinearOpMode {

    GamepadEx operator;

    private Servo clawRight;
    private Servo clawLeft;

    public static double clawOpen = 0.090;
    private double clawClosed = 0;

    private boolean rightOpen = false;
    private boolean leftOpen = false;





    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        operator = new GamepadEx(gamepad2);

        telemetry.addData("Status", "Initialized");
        telemetry.update();



        clawRight = hardwareMap.get(Servo.class, "clawRight");
        clawLeft = hardwareMap.get(Servo.class, "clawLeft");

        clawRight.setDirection(Servo.Direction.REVERSE);


        clawLeft.setPosition(0);
        clawRight.setPosition(0);


        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            operator.readButtons();


            if(operator.wasJustPressed(GamepadKeys.Button.B)){
                leftOpen = !leftOpen;
                clawLeft.setPosition(leftOpen ? clawOpen : clawClosed);
            }
            if(operator.wasJustPressed(GamepadKeys.Button.Y)){
                rightOpen = !rightOpen;
                clawRight.setPosition(rightOpen ? clawOpen : clawClosed);
            }


            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }

    }



}