package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class AutonomousFunctions extends MovementFunctions {

    /*
     * Method to perform a relative move, based on encoder counts.
     * Encoders are not reset as the move is based on the current position.
     * Move will stop if any of three conditions occur:
     * 1) Move gets to the desired position
     * 2) Move runs out of time
     * 3) Driver stops the OpMode running.
     */

    static final double COUNTS_PER_MOTOR_REV = 360; // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0; // No External Gearing.
    static final double WHEEL_DIAMETER_CM = 10.0; // For figuring circumference
    static final double COUNTS_PER_CM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_CM * 3.1415);
    static final double DRIVE_SPEED = 0.3;
    static final double TURN_SPEED = 0.3;

    final double SPEED_GAIN = 0.02; // Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.
                                    // (0.50 / 25.0)
    final double STRAFE_GAIN = 0.015; // Strafe Speed Control "Gain". eg: Ramp up to 25% power at a 25 degree Yaw
                                      // error. (0.25 / 25.0)
    final double TURN_GAIN = 0.01; // Turn Control "Gain". eg: Ramp up to 25% power at a 25 degree error. (0.25 /
                                   // 25.0)

    final double MAX_AUTO_SPEED = 0.5; // Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE = 0.5; // Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN = 0.3; // Clip the turn speed to this max value (adjust for your robot)

    final double DISTANCE = 10.0;

    public void alignToActualDetection(AprilTagDetection desiredTag) {
        double range = desiredTag.ftcPose.range;
        double heading = desiredTag.ftcPose.bearing;
        double yaw = desiredTag.ftcPose.yaw;

        // Use the speed and turn "gains" to calculate how we want the robot to move.
        // double drive = Range.clip(range * SPEED_GAIN, -MAX_AUTO_SPEED,
        // MAX_AUTO_SPEED);
        // double turn = Range.clip(heading * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN)
        // ;

        encoderTurn(yaw);

    }

    public void encoderDrive(double x, double y) {

        int targetFrontLeft = frontLeft.getCurrentPosition()
                - (int) (x * COUNTS_PER_CM)
                + (int) (y * COUNTS_PER_CM);
        int targetFrontRight = frontRight.getCurrentPosition()
                + (int) (x * COUNTS_PER_CM)
                + (int) (y * COUNTS_PER_CM);
        int targetBackLeft = backLeft.getCurrentPosition()
                + (int) (x * COUNTS_PER_CM)
                + (int) (y * COUNTS_PER_CM);
        int targetBackRight = backRight.getCurrentPosition()
                - (int) (x * COUNTS_PER_CM)
                + (int) (y * COUNTS_PER_CM);

        switchMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setTargetPosition(targetFrontLeft);
        frontRight.setTargetPosition(targetFrontRight);
        backLeft.setTargetPosition(targetBackLeft);
        backRight.setTargetPosition(targetBackRight);
        switchMotorModes(DcMotor.RunMode.RUN_TO_POSITION);

        MotorValues motorValues = new MotorValues(DRIVE_SPEED);

        applyMotorValues(motorValues);

        while ((frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy())
                && opModeIsActive()) {
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
