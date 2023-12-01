package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "AutonomousRedBackdrop", group = "Linear Opmode")
public class AutonomousRedBackdrop extends AutonoumousFunctions {
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        initialiseMecanum();
        initialiseArm();
        waitForStart();
        runtime.reset();

    }

}
