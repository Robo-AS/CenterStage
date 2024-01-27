package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Arrays;
import java.util.List;


@Config
public class Arm {
    private DcMotorEx circularMovementMotor, linearSlideMotor;
    private Servo servoClawAngle;

    static final double COUNTS_PER_MOTOR_REV_CIRCULAR = 384.5; //motor 435 rpm
    static final double COUNTS_PER_MOTOR_REV_LINEAR = 537.7; //motor 300 ceva rpm
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




//    List<Double> listOfLinearSlidePositions = Arrays.asList(0.0, 0.0, 90.0, 220.0);
    List<Double> listOfArmAngles = Arrays.asList(0.0, circularPos_3, circularPos_1, circularPos_2);
    List<Double> listOfClawAngles = Arrays.asList(0.0, 0.0, servoAngle_1, servoAngle_2);
    private int arm_position_index = 0;



    public static double circularPower = 1;


    public static double circularPos_3 = 15.0;
    public static double circularPos_1 = 123.0;
    public static double circularPos_2 = 116.7;




    public static double servoAngle_1 = 0.6;
    public static double servoAngle_2 = 0.61;



    public static int ticksToMoveUP = 110;
    public static int ticksToMoveDOWN = 150;
    public static double powerToMove = 0.9;

    enum States  {
        notLifting,
        liftArm,
        closeArm
    }

    States currentState=States.notLifting;

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


        //                if(gamepad.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)){
//                    armCircularMovement(circularPower, circularPos_1);
//                    servoClawAngle.setPosition(servoAngle_1);
//                }
//
//                if(gamepad.wasJustPressed(GamepadKeys.Button.DPAD_UP)){
//                    armCircularMovement(circularPower, circularPos_2);
//                    servoClawAngle.setPosition(servoAngle_2);
//                }
//
//                if(gamepad.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)){
//                    armCircularMovement(circularPower, circularPos_3);
//                    servoClawAngle.setPosition(0);
//                }
//
//
//                if(gamepad.wasJustPressed((GamepadKeys.Button.DPAD_DOWN))){
//                    armCircularMovement(circularPower, 0.0);
//                    servoClawAngle.setPosition(0);
//                }


        if(gamepad.wasJustPressed(GamepadKeys.Button.DPAD_UP)){
            arm_position_index = Math.min(3, arm_position_index+1);
            armCircularMovement(circularPower, listOfArmAngles.get(arm_position_index));
            servoClawAngle.setPosition(listOfClawAngles.get(arm_position_index));
        }


        if(gamepad.wasJustPressed((GamepadKeys.Button.DPAD_DOWN))){
            arm_position_index=Math.max(0, arm_position_index-1);
            armCircularMovement(circularPower, listOfArmAngles.get(arm_position_index));
            servoClawAngle.setPosition(listOfClawAngles.get(arm_position_index));
        }

        if(gamepad.isDown(GamepadKeys.Button.RIGHT_BUMPER) && linearSlideMotor.getCurrentPosition() <= 1100-ticksToMoveUP){
            linearSlideMotor.setTargetPosition(linearSlideMotor.getCurrentPosition() + ticksToMoveUP);
            linearSlideMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            linearSlideMotor.setPower(powerToMove);
        }

        if(gamepad.isDown(GamepadKeys.Button.LEFT_BUMPER) && linearSlideMotor.getCurrentPosition() >= ticksToMoveUP){
            linearSlideMotor.setTargetPosition(linearSlideMotor.getCurrentPosition() - ticksToMoveUP);
            linearSlideMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            linearSlideMotor.setPower(powerToMove);
        }

//        if(gamepad.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)){
//            if(currentState==States.notLifting){
//                currentState=States.liftArm;
//            }
//            else if(currentState==States.liftArm) {
//                currentState = States.closeArm;
//            }
//            else if(!linearSlideMotor.isBusy() || !circularMovementMotor.isBusy()){
//                //daca nu mere atunci e ca da isBusy mereu
//                //allternativ incearca sa vezi diferenta intre targetPosition si currentPosition si daca e mai mic de cat gen 5-6 ticks sau ceva
//
////
////                 if(Math.abs(linearSlideMotor.getCurrentPosition-linearSlideMotor.getTargetPosition())<6 &&
////                Math.abs(circularSlideMotor.getCurrentPosition-circularSlideMotor.getTargetPosition())<6)
//
//                if(currentState==States.liftArm){
//                    currentState=States.closeArm;
//                }else if(currentState==States.closeArm){
//                    currentState=States.notLifting;
//                }
//            }
//
//
//
//        }

//        switch (currentState){
//            case notLifting:
////                if(gamepad.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)){
////                    armCircularMovement(circularPower, circularPos_1);
////                    servoClawAngle.setPosition(servoAngle_1);
////                }
////
////                if(gamepad.wasJustPressed(GamepadKeys.Button.DPAD_UP)){
////                    armCircularMovement(circularPower, circularPos_2);
////                    servoClawAngle.setPosition(servoAngle_2);
////                }
////
////                if(gamepad.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)){
////                    armCircularMovement(circularPower, circularPos_3);
////                    servoClawAngle.setPosition(0);
////                }
////
////
////                if(gamepad.wasJustPressed((GamepadKeys.Button.DPAD_DOWN))){
////                    armCircularMovement(circularPower, 0.0);
////                    servoClawAngle.setPosition(0);
////                }
//
//
//                if(gamepad.wasJustPressed(GamepadKeys.Button.DPAD_UP)){
//                    arm_position_index = Math.min(3, arm_position_index+1);
//                    armCircularMovement(circularPower, listOfArmAngles.get(arm_position_index));
//                    servoClawAngle.setPosition(listOfClawAngles.get(arm_position_index));
//
//                }
//
//
//                if(gamepad.wasJustPressed((GamepadKeys.Button.DPAD_DOWN))){
//                    arm_position_index=Math.max(0, arm_position_index-1);
//                    armCircularMovement( circularPower, listOfArmAngles.get(arm_position_index));
//                    servoClawAngle.setPosition(listOfClawAngles.get(arm_position_index));
//                }
//
//                if(gamepad.isDown(GamepadKeys.Button.RIGHT_BUMPER) && linearSlideMotor.getCurrentPosition() <= 1100-ticksToMoveUP){
//                    linearSlideMotor.setTargetPosition(linearSlideMotor.getCurrentPosition() + ticksToMoveUP);
//                    linearSlideMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//                    linearSlideMotor.setPower(powerToMove);
//                }
//
//                if(gamepad.isDown(GamepadKeys.Button.LEFT_BUMPER) && linearSlideMotor.getCurrentPosition() >= ticksToMoveDOWN){
//                    linearSlideMotor.setTargetPosition(linearSlideMotor.getCurrentPosition() - ticksToMoveDOWN);
//                    linearSlideMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//                    linearSlideMotor.setPower(powerToMove);
//                }
//
//                break;
//
//
//            case liftArm:
//                armCircularMovement(circularPower, 120.0);
//                armLinearMovement(0.1, 120.0);
//                break;
//            case closeArm:
//                //AICI AVEM PROBLEMA CA SA CONSTRACTA IN CONTINUU
//                armLinearMovement(0.1, 50.0);
//                break;
//        }


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
        circularMovementMotor.setTargetPosition(targetTicks );
        circularMovementMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        circularMovementMotor.setPower(power);
    }

}