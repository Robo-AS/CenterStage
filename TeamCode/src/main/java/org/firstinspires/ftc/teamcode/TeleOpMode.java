package org.first.inspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opMode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="TeleOpMode", group="Linear OpMode");

public class TeleOpMode extends UsefulFunctions {

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        Initialise();

        waitForStart();

        //nam idee ce dracu ar trebui sa pun aici

        while(opModeIsActive()){
            TeleOpDrive();
        }
    }


}

