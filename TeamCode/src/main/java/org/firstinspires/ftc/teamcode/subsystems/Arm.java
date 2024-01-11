package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
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

public class Arm {
    private DcMotor circularMovementMotor, linearSlideMotor;
    private Servo servoClawAngle;

    static final double COUNTS_PER_MOTOR_REV_CIRCULAR = 145.1;  //motor 1150 rpm
    static final double COUNTS_PER_MOTOR_REV_LINEAR = 384.5; //motor 435 rpm
    static final  double DRIVE_GEAR_REDUCTION_LINEAR = 1;
    static final double DRIVE_GEAR_REDUCTION_CIRCULAR = 28;
    public static double PULLEY_CIRCUMFERENCE_MM = 35.65 * Math.PI;   //aprox. 122 mm
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


    List<Double> listOfLinearSlidePositions = Arrays.asList(0.0, 220.0); //trebuie determinati parametrii ca lumea
    List<Double> listOfArmAngles = Arrays.asList(0.0, 90.0); //trebuie determinati parametrii ca lumea
    List<Double> listOfClawAngles = Arrays.asList(0.0, 0.25);



    private double linearSlidePower = 0.4;

    private double circularPower = 0.8;
    private double linearSlideMult = 222.0/220.0;

    private int arm_position_index = 0;

    public Arm(HardwareMap hardwareMap){
        linearSlideMotor = hardwareMap.get(DcMotor.class, "linearSlideMotor");
        circularMovementMotor = hardwareMap.get(DcMotor.class, "circularMovementMotor");
        servoClawAngle = hardwareMap.get(Servo.class, "servoClawAngle");

        circularMovementMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        circularMovementMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        circularMovementMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        linearSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


    public void teleop(GamepadEx gamepad, Telemetry telemetry){

        if(gamepad.wasJustPressed(GamepadKeys.Button.DPAD_UP)){
            arm_position_index = Math.min(2, arm_position_index+1);
            armLinearMovement(linearSlidePower, listOfLinearSlidePositions.get(arm_position_index));
            armCircularMovement( circularPower, listOfArmAngles.get(arm_position_index));
            servoClawAngle.setPosition(listOfClawAngles.get(arm_position_index));
        }

        if(gamepad.wasJustPressed((GamepadKeys.Button.DPAD_DOWN))){
            arm_position_index=Math.max(0, arm_position_index-1);
            armLinearMovement(linearSlidePower, listOfLinearSlidePositions.get(arm_position_index));
            armCircularMovement( circularPower, listOfArmAngles.get(arm_position_index));
            servoClawAngle.setPosition(listOfClawAngles.get(arm_position_index));
        }

        if (circularMovementMotor.getCurrentPosition()>circularMovementMotor.getTargetPosition()){
            int dist = circularMovementMotor.getCurrentPosition()-circularMovementMotor.getTargetPosition();
            double coeff = dist;
            coeff = coeff/180.0;
            circularMovementMotor.setPower(coeff);  //daca nu merge doar pune gen 0.2 sa uceva
        }


        telemetry.addData("linear slide",linearSlideMotor.getCurrentPosition());
        telemetry.addData("wristangle", servoClawAngle.getPosition());


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



    //unde naiba pun listele cu parametrii?//le-am pus aici

}

