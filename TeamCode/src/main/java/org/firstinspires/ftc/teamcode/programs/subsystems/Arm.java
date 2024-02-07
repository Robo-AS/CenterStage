package org.firstinspires.ftc.teamcode.programs.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Arrays;
import java.util.List;


@Config
public class Arm {
    private DcMotorEx circularMovementMotor, linearSlideMotor;
    private Servo servoClawAngle;

    static final double COUNTS_PER_MOTOR_REV_CIRCULAR = 145.1; //motor 1150 rpm
    static final double COUNTS_PER_MOTOR_REV_LINEAR = 537.7; //motor 312 rpm
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




    List<Double> listOfArmAngles = Arrays.asList(0.0, 0.0, circularPos_3);
    List<Double> listOfClawAngles = Arrays.asList(0.0, servoAngle_1, servoAngle_3);
    private int arm_position_index = 0;



    public static int ticksToMove = 110;
    public static double linearPower = 1;
    public static double circularPower = 0.5;

    public static double circularPos_2 = 123.0;
    public static double circularPos_3 = 115.0;
    public static double hangAngle = 50.0;


    public static double servoAngle_1 = 0.62;
    public static double servoAngle_3 = 0.65;

    public boolean pos2 = false;







    public Arm(HardwareMap hardwareMap){
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

    }


    public void teleop(GamepadEx gamepad, Telemetry telemetry){

        if(gamepad.isDown(GamepadKeys.Button.RIGHT_BUMPER) && linearSlideMotor.getCurrentPosition() <= 2065- ticksToMove){
            linearSlideMotor.setTargetPosition(linearSlideMotor.getCurrentPosition() + ticksToMove);
            linearSlideMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            linearSlideMotor.setPower(linearPower);
        }

        if(gamepad.isDown(GamepadKeys.Button.LEFT_BUMPER) && linearSlideMotor.getCurrentPosition() >= ticksToMove){
            linearSlideMotor.setTargetPosition(linearSlideMotor.getCurrentPosition() - ticksToMove);
            linearSlideMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            linearSlideMotor.setPower(linearPower);
        }




        if(gamepad.wasJustPressed(GamepadKeys.Button.DPAD_UP)){
            arm_position_index = Math.min(2, arm_position_index+1);
            armCircularMovement(circularPower, listOfArmAngles.get(arm_position_index));
            servoClawAngle.setPosition(listOfClawAngles.get(arm_position_index));
        }


        if(gamepad.wasJustPressed((GamepadKeys.Button.DPAD_DOWN))){
            if(pos2) {
                arm_position_index = 2;
                pos2 = false;
            }

            arm_position_index=Math.max(0, arm_position_index-1);
            armCircularMovement(circularPower, listOfArmAngles.get(arm_position_index));
            servoClawAngle.setPosition(listOfClawAngles.get(arm_position_index));
        }

        if(gamepad.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)){
            pos2 = true;
            armCircularMovement(circularPower, circularPos_2);
            servoClawAngle.setPosition(servoAngle_1);
        }

        if(gamepad.wasJustPressed(GamepadKeys.Button.X)){
            armCircularMovement(circularPower, hangAngle);
            servoClawAngle.setPosition(0.0);
        }


        telemetry.addData("linearSlidePos", linearSlideMotor.getCurrentPosition());
        telemetry.addData("circularMotionMotor", circularMovementMotor.getCurrentPosition());

    }

    private void armLinearMovement(double power, double linear_slide_mm){
        int targetTicks = (int) (linear_slide_mm * COUNTS_PER_MM);
        linearSlideMotor.setTargetPosition(targetTicks);
        linearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlideMotor.setPower(power);
    }


    public void armCircularMovement(double power, double degrees){
        int targetTicks = (int)(degrees * COUNTS_PER_DEGREE);
        circularMovementMotor.setTargetPosition(targetTicks);
        circularMovementMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        circularMovementMotor.setPower(power);
    }

}