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

    static final double COUNTS_PER_MOTOR_REV_CIRCULAR = 145.1;  //motor 1150 rpm
    static final double COUNTS_PER_MOTOR_REV_LINEAR = 1425.1; //motor 117 rpm
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




    List<Double> listOfLinearSlidePositions = Arrays.asList(0.0, 0.0, 50.0, 220.0);
    List<Double> listOfArmAngles = Arrays.asList(0.0, 25.0, 154.0, 147.0);
    List<Double> listOfClawAngles = Arrays.asList(0.0, 0.0, 0.5, 0.5);



    public static double linearSlidePower = 0.5;
    public static double circularPowerUP = 0.5;

    public static double circularPowerDOWN = 0.4;
    private double linearSlideMult = 222.0/220.0;

    private int arm_position_index = 0;

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

        circularMovementMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        circularMovementMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        circularMovementMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);


        linearSlideMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        linearSlideMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        servoClawAngle.setPosition(0);
    }


    public void teleop(GamepadEx gamepad, Telemetry telemetry){
        //AM INCERCAT AICI SA II DAM PUTERE 0 BRATULUI CAND SE AFLA LA POZITIA 0 - E IDEE BUNA MERITA INCERCAT
//        if(circularMovementMotor.getCurrentPosition() == listOfArmAngles.get(arm_position_index) && arm_position_index == 0){
//            armCircularMovement(0, listOfArmAngles.get(arm_position_index));
//        }
        if(gamepad.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)){
            if(currentState==States.notLifting){
                currentState=States.liftArm;
            }
            else if(!linearSlideMotor.isBusy() || !circularMovementMotor.isBusy()){
                //daca nu mere atunci e ca da isBusy mereu
                //allternativ incearca sa vezi diferenta intre targetPosition si currentPosition si daca e mai mic de cat gen 5-6 ticks sau ceva

//
//                 if(Math.abs(linearSlideMotor.getCurrentPosition-linearSlideMotor.getTargetPosition())<6 &&
//                Math.abs(circularSlideMotor.getCurrentPosition-circularSlideMotor.getTargetPosition())<6)

                if(currentState==States.liftArm){
                    currentState=States.closeArm;
                }else if(currentState==States.closeArm){
                    currentState=States.notLifting;
                }
            }



        }

        switch (currentState){
            case notLifting:
                telemetry.addData(">", "NOT_LIFTING");
                telemetry.update();
                if(gamepad.wasJustPressed(GamepadKeys.Button.DPAD_UP)){
                    arm_position_index = Math.min(3, arm_position_index+1);
                    armCircularMovement( circularPowerUP, listOfArmAngles.get(arm_position_index));
                    servoClawAngle.setPosition(listOfClawAngles.get(arm_position_index));
                    armLinearMovement(linearSlidePower, listOfLinearSlidePositions.get(arm_position_index));

                }


                if(gamepad.wasJustPressed((GamepadKeys.Button.DPAD_DOWN))){
                    arm_position_index=Math.max(0, arm_position_index-1);
                    armCircularMovement( circularPowerDOWN, listOfArmAngles.get(arm_position_index));
                    servoClawAngle.setPosition(listOfClawAngles.get(arm_position_index));
                    armLinearMovement(linearSlidePower, listOfLinearSlidePositions.get(arm_position_index));

                }

//                if (circularMovementMotor.getCurrentPosition()>circularMovementMotor.getTargetPosition()){
//                    int dist = circularMovementMotor.getCurrentPosition()-circularMovementMotor.getTargetPosition();
//                    double coeff = dist;
//                    //coeff = coeff/COUNTS_PER_DEGREE/180.0+0.05;
//                    coeff = 0.2;
//                    circularMovementMotor.setPower(0.2);                                                                  //daca nu merge doar pune gen 0.2 sa uceva
//                }
                break;


            case liftArm:
                armCircularMovement(circularPowerUP, 120.0);
                armLinearMovement(0.01, 120.0);
                break;
            case closeArm:
                //AICI AVEM PROBLEMA CA SA CONSTRACTA IN CONTINUU
                armLinearMovement(linearSlidePower, 50.0 );

                break;
        }

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