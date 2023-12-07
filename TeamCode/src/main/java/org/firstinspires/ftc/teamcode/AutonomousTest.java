package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "AutonomousTest", group = "Linear Opmode")
public class AutonomousTest extends AutonomousFunctions {
    private ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() {

        initialiseMecanum();
        initialiseArm();
        waitForStart();

        encoderDrive(12,0);
        encoderDrive(0, 12);
        encoderDrive(-12,-12);

        runtime.reset();

    }

}
