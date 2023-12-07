package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


public class AutonomousFunctions extends MovementFunctions {

    /* 
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the OpMode running.
     */

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_CM   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_CM         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_CM * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;


    public void encoderDrive(double x, double y) {

        int targetFrontLeft = frontLeft.getCurrentPosition()
                        + (int)(x * COUNTS_PER_CM)
                        + (int)(y * COUNTS_PER_CM);
        int targetFrontRight = frontRight.getCurrentPosition()
                        - (int)(x * COUNTS_PER_CM)
                        - (int)(y * COUNTS_PER_CM);
        int targetBackLeft = backLeft.getCurrentPosition()
                        - (int)(x * COUNTS_PER_CM)
                        + (int)(y * COUNTS_PER_CM);
        int targetBackRight = backRight.getCurrentPosition()
                        + (int)(x * COUNTS_PER_CM)
                        - (int)(y * COUNTS_PER_CM);      
        
        switchMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setTargetPosition(targetFrontLeft);
        frontRight.setTargetPosition(targetFrontRight);
        backLeft.setTargetPosition(targetBackLeft);
        backRight.setTargetPosition(targetBackRight);
        switchMotorModes(DcMotor.RunMode.RUN_TO_POSITION);
        
        MotorValues motorValues = new MotorValues(DRIVE_SPEED);

        applyMotorValues(motorValues);

        while ((frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()) && opModeIsActive()) {
            telemetry.addData("frontLeft", frontLeft.getCurrentPosition());
            telemetry.addData("frontRight", frontRight.getCurrentPosition());
            telemetry.addData("backLeft", backLeft.getCurrentPosition());
            telemetry.addData("backRight", backRight.getCurrentPosition());
            telemetry.update();
        }

    }

    public void encoderTurn(double degrees) {

        applyMotorValues(new MotorValues(0));
        switchMotorModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        

    }

}
