package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


//@TeleOp(name="MecanumDrive", group="Linear OpMode")
public class MecanumDrive extends LinearOpMode {
    DcMotor frontRight, frontLeft, backRight, backLeft;

    public void InitialiseMotors(){
        frontLeft=hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight=hardwareMap.get(DcMotor.class, "frontRight");
        backLeft=hardwareMap.get(DcMotor.class, "backLeft");
        backRight=hardwareMap.get(DcMotor.class, "backRight");


        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void MotorPower(){
        //each of these variable take their values from the joystick input
        double vertical = 0;    //variable for vertical movement
        double horizontal = 0;  //variable for horizontal movement
        double pivot = 0;       //variable for rotation


        vertical = -gamepad1.left_stick_y;   // vertical = to the value from left stick, y axe
        horizontal = gamepad1.left_stick_x; // horizontal = to the value from the left stick, x axe
        pivot = gamepad1.right_stick_x;     //pivot = to the value from the right stick, x axe

        double power_frontLeft = vertical - horizontal + pivot;
        double power_frontRight = vertical - horizontal - pivot;
        double power_backLeft = -vertical - horizontal - pivot;
        double power_backRight = -vertical - horizontal + pivot;

        if(gamepad1.left_bumper){
            power_frontLeft/=2;
            power_frontRight/=2;
            power_backLeft/=2;
            power_backRight/=2;
        }
        //Setting power to the motors(i don't understand the formulas but not so important for now)
        frontLeft.setPower(power_frontLeft);
        frontRight.setPower(power_frontRight);
        backLeft.setPower(power_backLeft);
        backRight.setPower(power_backRight);

    }
    @Override
    public void runOpMode () throws InterruptedException {
    }


}
