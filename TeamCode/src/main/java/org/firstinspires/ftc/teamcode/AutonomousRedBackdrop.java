package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name = "AutonomousRedBackdrop", group = "Linear Opmode")
public class AutonomousRedBackdrop extends Useful_Funtions {
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode(){
        InitialiseMecanum();
        InitialiseArm();


        waitForStart();
        runtime.reset();
    }




}
