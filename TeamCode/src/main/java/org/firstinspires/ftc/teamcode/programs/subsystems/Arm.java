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
    static final double DRIVE_GEAR_REDUCTION_CIRCULAR = 28;


    static final double COUNTS_PER_GEAR_REV = COUNTS_PER_MOTOR_REV_CIRCULAR * DRIVE_GEAR_REDUCTION_CIRCULAR;  //4062.8 ticks
    public static double COUNTS_PER_DEGREE = COUNTS_PER_GEAR_REV/360;                   //aprox 11.285  tiks/degree


    // 145.1 ticks - o revolutie a motorului
    // 145.1 * 28  - 28 de revolutii ale motorului
    //             - 28 de revolutii - revolutie worm gear
    //                               - revolutie worm gear - 360 de grade brat
    // 145.1 * 28 - 360 de grade brat
    // 11.285     - 1 grad brat
    //




    List<Double> listOfArmAngles = Arrays.asList(0.0, 0.0, circularPos_2, circularPos_3);
    List<Double> listOfClawAngles = Arrays.asList(servoAngle_0, servoAngle_1, servoAngle_1, servoAngle_3);

    List<Integer> ticksToMoveCase = Arrays.asList(0, 0, ticks2, ticks3);



    private int arm_position_index = 0;





    public static double linearPower = 1; //1
    public static double circularPower = 0.3;//0.8

    public static double initAngleTeleOp = -37.0;   //corectionAngle o sa aiba valoare negativa, deci facand suma cu el defapt scadem din unghiurile anterioare
//    public static double correctionAngleAuto = -28.0;
    public static double correctionAngleAuto = -25.0;

    public static double circularPos_2 = 120.0;
    public static double circularPos_3 = 100.0;
    public static double hangAngle = 55.0;




    public static double servoAngle_0 = 0.035;

    public static double servoAngle_1 = 0.62;
    public static double servoAngle_3 = 0.65;



    public static int ticksToMoveLinear = 110;
    public static int ticks2 = -193;//-181;
    public static int ticks3 = -183;//-170






    public Arm(HardwareMap hardwareMap){
        linearSlideMotor = hardwareMap.get(DcMotorEx.class, "linearSlideMotor");
        circularMovementMotor = hardwareMap.get(DcMotorEx.class, "circularMovementMotor");
        servoClawAngle = hardwareMap.get(Servo.class, "servoClawAngle");

        circularMovementMotor.setDirection(DcMotorEx.Direction.REVERSE);
        circularMovementMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        circularMovementMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);


        linearSlideMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        linearSlideMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        servoClawAngle.setDirection(Servo.Direction.REVERSE);
        servoClawAngle.setPosition(servoAngle_0);
    }





    public void teleop(GamepadEx gamepad, Telemetry telemetry){

        if(gamepad.isDown(GamepadKeys.Button.RIGHT_BUMPER) && linearSlideMotor.getCurrentPosition() <= 2065- ticksToMoveLinear){
            linearSlideMotor.setTargetPosition(linearSlideMotor.getCurrentPosition() + ticksToMoveLinear);
            linearSlideMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            linearSlideMotor.setPower(linearPower);
        }

        if(gamepad.isDown(GamepadKeys.Button.LEFT_BUMPER) && linearSlideMotor.getCurrentPosition() >= ticksToMoveLinear){
            linearSlideMotor.setTargetPosition(linearSlideMotor.getCurrentPosition() - ticksToMoveLinear);
            linearSlideMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            linearSlideMotor.setPower(linearPower);
        }




        if(gamepad.wasJustPressed(GamepadKeys.Button.DPAD_UP)){
            if(arm_position_index == 1)
                arm_position_index++;

            arm_position_index = Math.min(3, arm_position_index+1);
            armCircularMovement(circularPower, listOfArmAngles.get(arm_position_index));
            armLinearMovement(linearPower, ticksToMoveCase.get(arm_position_index));
            servoClawAngle.setPosition(listOfClawAngles.get(arm_position_index));

        }


        if(gamepad.wasJustPressed((GamepadKeys.Button.DPAD_DOWN))){
            if(arm_position_index == 3)
                arm_position_index--;

            arm_position_index=Math.max(0, arm_position_index-1);
            armCircularMovement(circularPower, listOfArmAngles.get(arm_position_index));
            armLinearMovement(linearPower, ticksToMoveCase.get(arm_position_index));
            servoClawAngle.setPosition(listOfClawAngles.get(arm_position_index));
        }

        if(gamepad.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)){
            arm_position_index = 2;
            armCircularMovement(circularPower, listOfArmAngles.get(arm_position_index));
            armLinearMovement(linearPower, ticksToMoveCase.get(arm_position_index));
            servoClawAngle.setPosition(listOfClawAngles.get(arm_position_index));
        }

        if(gamepad.wasJustPressed(GamepadKeys.Button.X)){
            armCircularMovement(circularPower, hangAngle);
            servoClawAngle.setPosition(servoAngle_0);
        }


        telemetry.addData("linearSlidePos", linearSlideMotor.getCurrentPosition());
        telemetry.addData("circularMotionMotor", circularMovementMotor.getCurrentPosition());
        telemetry.addData("index", arm_position_index);
    }


    public void initAutonomous(){
        servoClawAngle.setPosition(servoAngle_0);
        armCircularMovement(circularPower, correctionAngleAuto);
    }


    public void autonomousAngleUp(){
        armCircularMovement(circularPower, circularPos_2 + correctionAngleAuto);
        servoClawAngle.setPosition(servoAngle_1);
    }

    public void autonomousAngleDown(){
        armCircularMovement(circularPower, initAngleTeleOp);
        servoClawAngle.setPosition(servoAngle_0);
    }


    private void armLinearMovement(double power, int targetTicks){
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