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
@TeleOp(name = "Arm_TEST2", group= "Linear Opmode")
public class Arm_TEST2 extends LinearOpMode {

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


    // 145.1 ticks - o revolutie a motorului
    // 145.1 * 28  - 28 de revolutii ale motorului
    //             - 28 de revolutii - revolutie worm gear
    //                               - revolutie worm gear - 360 de grade brat
    // 145.1 * 28 - 360 de grade brat
    // 11.285     - 1 grad brat




    public static double circularPower = 0.5;

    public static double circularPos_1 = 123.0;
    public static double circularPos_2 = 115.0;


//    public static int linearTicks_1 = 36;
//    public static int linearTicks_2 = 10;

    public static double servoAngle_1 = 0.62;
    public static double servoAngle_2 = 0.65;




//    public static int divider = 4;

    public static int ticksToMove = 110;
    public static double powerToMove = 1;

    public static double CLAW_INIT = 0;


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
        servoClawAngle.setPosition(CLAW_INIT);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            operator.readButtons();

            if(operator.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)){
                armCircularMovement(circularPower, circularPos_1);
                servoClawAngle.setPosition(servoAngle_1);

            }

            if(operator.wasJustPressed(GamepadKeys.Button.DPAD_UP)){
                armCircularMovement(circularPower, circularPos_2);
                servoClawAngle.setPosition(servoAngle_2);

            }


            if(operator.wasJustPressed((GamepadKeys.Button.DPAD_DOWN))){
                armCircularMovement(circularPower, 0.0);
                servoClawAngle.setPosition(CLAW_INIT);
            }






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

//    private void armLinearMovement(double power, int targetTicks){
////        int targetTicks = (int) (linear_slide_mm * COUNTS_PER_MM);
//        linearSlideMotor.setTargetPosition(targetTicks);
//        linearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        linearSlideMotor.setPower(power);
//    }


    public void armCircularMovement(double power, double degrees){
        int targetTicks = (int)(degrees * COUNTS_PER_DEGREE);
        circularMovementMotor.setTargetPosition(targetTicks );
        circularMovementMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        circularMovementMotor.setPower(power);
    }





//    public void armLinearMovement_2(double linearSlidePower){
//
//        if(linearSlideMotor.getCurrentPosition() >= LINEAR_SLIDE_MAX_POSITION && linearSlidePower > 0){
//            linearSlideMotor.setPower(REST_POWER);
//            return;
//        }
//        else if(linearSlideMotor.getCurrentPosition() <= 0 && linearSlidePower < 0){
//            linearSlideMotor.setPower(REST_POWER);
//            return;
//        }
//        else if(linearSlidePower==0 && linearSlideMotor.getCurrentPosition()!=0){
//            linearSlideMotor.setPower(REST_POWER);
//        }
//        else linearSlideMotor.setPower(linearSlidePower/divider);
//    }


}