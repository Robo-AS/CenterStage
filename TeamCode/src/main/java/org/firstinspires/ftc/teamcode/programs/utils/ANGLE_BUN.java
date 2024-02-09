package org.firstinspires.ftc.teamcode.programs.utils;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name = "ANGLE_BUN", group= "Linear Opmode")
public class ANGLE_BUN extends LinearOpMode {

    GamepadEx operator;

    private DcMotorEx circularMovementMotor, linearSlideMotor;
    private Servo servoClawAngle;

    static final double COUNTS_PER_MOTOR_REV_CIRCULAR = 145.1;  //motor 1150 rpm

    static final double DRIVE_GEAR_REDUCTION_CIRCULAR = 28;



    static final double COUNTS_PER_GEAR_REV = COUNTS_PER_MOTOR_REV_CIRCULAR * DRIVE_GEAR_REDUCTION_CIRCULAR;  //4062.8 ticks
    public static double COUNTS_PER_DEGREE = COUNTS_PER_GEAR_REV/360;                   //aprox 11.285  tiks/degree


    // 145.1 ticks - o revolutie a motorului
    // 145.1 * 28  - 28 de revolutii ale motorului
    //             - 28 de revolutii - revolutie worm gear
    //                               - revolutie worm gear - 360 de grade brat
    // 145.1 * 28 - 360 de grade brat
    // 11.285     - 1 grad brat




    public static double circularPower = 0.5;

    public static double ANGLE = 35.0;

    private static double INIT = 0.0;


    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        operator = new GamepadEx(gamepad2);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        linearSlideMotor = hardwareMap.get(DcMotorEx.class, "linearSlideMotor");
        circularMovementMotor = hardwareMap.get(DcMotorEx.class, "circularMovementMotor");
        servoClawAngle = hardwareMap.get(Servo.class, "servoClawAngle");

        circularMovementMotor.setDirection(DcMotorEx.Direction.REVERSE);
        circularMovementMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        circularMovementMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);



        linearSlideMotor.setDirection(DcMotorEx.Direction.REVERSE);
        linearSlideMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        linearSlideMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);


        servoClawAngle.setDirection(Servo.Direction.REVERSE);


        waitForStart();

        runtime.reset();






        while (opModeIsActive()) {
            operator.readButtons();

            if(operator.wasJustPressed(GamepadKeys.Button.DPAD_UP)){
                armCircularMovement(circularPower,ANGLE);
            }

            if(operator.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)){
                armCircularMovement(circularPower, INIT);
            }


            telemetry.addData("circularMotionMotor", circularMovementMotor.getCurrentPosition());
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }

    }




    public void armCircularMovement(double power, double degrees){
        int targetTicks = (int)(degrees * COUNTS_PER_DEGREE);
        circularMovementMotor.setTargetPosition(targetTicks );
        circularMovementMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        circularMovementMotor.setPower(power);
    }


}