package org.firstinspires.ftc.teamcode.programs.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static org.firstinspires.ftc.teamcode.programs.utils.Constants.CONTROLLER_DEADZONE;
import static org.firstinspires.ftc.teamcode.programs.utils.Constants.ROBOT_SPEED;
import java.util.Arrays;
import java.util.List;

/**
 * Robot centric drive is a drive system that allows the robot to move in a direction relative to the robot
 *
 * @version 1.0
 */
@Config
public class Mecanum {

    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private List<DcMotorEx> motors;

    double reverse = 1.0;
    public static double powerReduction = 9;



    public Mecanum(HardwareMap hardwareMap) {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);
        for (DcMotorEx motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
    }




    public void teleop(GamepadEx gamepad, Telemetry telemetry) {
        double x = -gamepad.getLeftX();
        double y = -gamepad.getLeftY();
        double r = -gamepad.getRightX();

        x = addons(x) * reverse;
        y = addons(y) * reverse;
        r = addons(r);

        double normalizer = Math.max(Math.abs(x) + Math.abs(y) + Math.abs(r), 1);

        double leftFrontPower = (y + x + r) / normalizer;
        double rightFrontPower = (y - x - r) / normalizer;
        double leftRearPower = (y - x + r) / normalizer;
        double rightRearPower = (y + x - r) / normalizer;

        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftRear.setPower(leftRearPower);
        rightRear.setPower(rightRearPower);

        //Slow motion
        if(gamepad.isDown(GamepadKeys.Button.LEFT_BUMPER)){
            leftFront.setPower(leftFrontPower/powerReduction);
            rightFront.setPower(rightFrontPower/powerReduction);
            leftRear.setPower(leftRearPower/powerReduction);
            rightRear.setPower(rightRearPower/powerReduction);
        }




    }

    public double addons(double value) {
        if (Math.abs(value) < CONTROLLER_DEADZONE) return 0;
        return value * ROBOT_SPEED;
    }



    public void setReverse(boolean isReverse) {
        if (isReverse) reverse = -1.0;
        else reverse = 1.0;
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addLine("---ROBOT CENTRIC DRIVE---");

        telemetry.addData("Direction Multiplier: ", reverse);
        telemetry.addData("Speed Multiplier: ", ROBOT_SPEED);

        telemetry.addData("LeftRear Position: ", leftRear.getCurrentPosition());
        telemetry.addData("RightRear Position: ", rightRear.getCurrentPosition());
        telemetry.addData("LeftFront Position: ", leftFront.getCurrentPosition());
        telemetry.addData("RightFront Position: ", rightFront.getCurrentPosition());

        telemetry.addData("LeftRear Power: ", leftRear.getPower());
        telemetry.addData("RightRear Power: ", rightRear.getPower());
        telemetry.addData("LeftFront Power: ", leftFront.getPower());
        telemetry.addData("RightFront Power: ", rightFront.getPower());
    }

}
