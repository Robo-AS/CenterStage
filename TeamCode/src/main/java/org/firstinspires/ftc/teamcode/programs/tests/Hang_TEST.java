package org.firstinspires.ftc.teamcode.programs.tests;

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
//@TeleOp(name = "Hang_TEST", group= "Linear Opmode")
public class Hang_TEST extends LinearOpMode {

    GamepadEx operator;

    private DcMotorEx circularMovementMotor, linearSlideMotor;
    private Servo servoClawAngle;

    static final double COUNTS_PER_MOTOR_REV_CIRCULAR = 145.1;  //motor 1150 rpm
    static final double COUNTS_PER_MOTOR_REV_LINEAR = 537.7;
    static final  double DRIVE_GEAR_REDUCTION_LINEAR = 1;
    static final double DRIVE_GEAR_REDUCTION_CIRCULAR = 28;
    public static double PULLEY_CIRCUMFERENCE_MM = 112;   //aprox. 122 mm
    static final double COUNTS_PER_PULLEY_REV = COUNTS_PER_MOTOR_REV_LINEAR * DRIVE_GEAR_REDUCTION_LINEAR; //751.8 ticks
    static final double COUNTS_PER_MM = COUNTS_PER_PULLEY_REV / PULLEY_CIRCUMFERENCE_MM; //aprox 6.162 ticks/mm

    static final double COUNTS_PER_GEAR_REV = COUNTS_PER_MOTOR_REV_CIRCULAR * DRIVE_GEAR_REDUCTION_CIRCULAR;  //4062.8 ticks
    public static double COUNTS_PER_DEGREE = COUNTS_PER_GEAR_REV/360;                   //aprox 11.285  tiks/degree







    public static double circularPower = 0.5;

    public static double circularPos = 50;

    public static double servoPos = 0.0;








    public static int divider = 4;

    public static int ticksToMove = 80;
    public static double powerToMove = 0.9;


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
        servoClawAngle.setPosition(0);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            operator.readButtons();


            if(operator.wasJustPressed(GamepadKeys.Button.X)){
                armCircularMovement(circularPower, circularPos);
                servoClawAngle.setPosition(servoPos);
            }


            if(operator.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)){
                armCircularMovement(circularPower, 0.0);
                servoClawAngle.setPosition(0);

            }



            //PT ASTEA INCEARCA STOP_AND_RESET_ENCODER DE FIECARE DATA CAND APESI DPAD

            if(operator.isDown(GamepadKeys.Button.RIGHT_BUMPER) && linearSlideMotor.getCurrentPosition() <= 2065-ticksToMove){
                linearSlideMotor.setTargetPosition(linearSlideMotor.getCurrentPosition() + ticksToMove);
                linearSlideMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                linearSlideMotor.setPower(powerToMove);
            }

            if(operator.isDown(GamepadKeys.Button.LEFT_BUMPER) && linearSlideMotor.getCurrentPosition() >= ticksToMove){
                linearSlideMotor.setTargetPosition(linearSlideMotor.getCurrentPosition() - ticksToMove);
                linearSlideMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                linearSlideMotor.setPower(powerToMove);
            }

            telemetry.addData("linearSlidePos", linearSlideMotor.getCurrentPosition());
            telemetry.addData("circularMotionMotor", circularMovementMotor.getCurrentPosition());
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }

    }

    private void armLinearMovement(double power, int targetTicks){
        linearSlideMotor.setTargetPosition(targetTicks);
        linearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlideMotor.setPower(power);
    }


    public void armCircularMovement(double power, double degrees){
        int targetTicks = (int)(degrees * COUNTS_PER_DEGREE);
        circularMovementMotor.setTargetPosition(targetTicks );
        circularMovementMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        circularMovementMotor.setPower(power);
    }







}