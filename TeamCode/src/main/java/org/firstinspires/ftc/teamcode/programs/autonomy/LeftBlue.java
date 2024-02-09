package org.firstinspires.ftc.teamcode.programs.autonomy;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.programs.subsystems.Arm;
import org.firstinspires.ftc.teamcode.programs.detection.GameElementDetection;
import org.firstinspires.ftc.teamcode.programs.subsystems.Claw;
import org.firstinspires.ftc.teamcode.programs.subsystems.Drone;
import org.firstinspires.ftc.teamcode.programs.subsystems.Mecanum;
import org.firstinspires.ftc.teamcode.programs.utils.AutonomousConstants;
import org.firstinspires.ftc.teamcode.programs.utils.Constants;
import org.firstinspires.ftc.teamcode.programs.utils.TrajectoryFactory;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

import java.util.List;

@Autonomous(name = "Left Blue")
public class LeftBlue extends LinearOpMode {
    GameElementDetection detection;
    GameElementDetection.Position position;

    Claw claw;
    Arm arm;

    @Override
    public void runOpMode()  throws InterruptedException {

        claw = new Claw(hardwareMap);
        arm = new Arm(hardwareMap);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        detection = new GameElementDetection();
        detection.init(hardwareMap, true);

        FtcDashboard.getInstance().startCameraStream(detection.getCamera(), 30);

        while (!opModeIsActive() && !isStopRequested()) {
            position = detection.getPosition();
            telemetry.addData("Position", detection.getPosition());
            telemetry.update();
        }

        detection.close();

        List<TrajectorySequence> trajectory = TrajectoryFactory.createTrajectory(drive, position, telemetry, true, true);

        waitForStart();

        arm.initAutonomous();

        drive.followTrajectorySequence(trajectory.get(0));
//        telemetry.addData("angle: ", drive.getPoseEstimate().getHeading());
//        telemetry.addData("angle: ", drive.getPoseEstimate().getX());
//        telemetry.addData("angle: ", drive.getPoseEstimate().getY());
//        telemetry.update();

        sleep(AutonomousConstants.SLEEP);
        claw.autonomousOpenRightClaw();
        sleep(AutonomousConstants.SLEEP);


        drive.followTrajectorySequence(trajectory.get(1));
        sleep(AutonomousConstants.SLEEP);
        arm.autonomousAngleUp();
        sleep(AutonomousConstants.SLEEP);
        claw.autonomousOpenLeftClaw();

//        drive.followTrajectorySequence(trajectory.get(2));



        while (!isStopRequested() && opModeIsActive()) {
            telemetry.update();
        }
    }
}