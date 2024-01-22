package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Mecanum;
import com.qualcomm.robotcore.util.ElapsedTime;

//@Autonomous(name = "AUTONOMOUS")
public class AUTONOMOUS extends LinearOpMode {
    // Drive function with 3 parameters

    private ElapsedTime runtime = new ElapsedTime();

    Mecanum drive ;

    Arm arm;

    Claw claw;

    private void conduX(double x) {

        drive.setPoseEstimate(new Pose2d(0,0,0));
        Pose2d poseEstimate = drive.getPoseEstimate();
        Pose2d targetPose = new Pose2d(x,0,0);
        double dx = targetPose.getX()-poseEstimate.getX();
        while (Math.abs(dx)>0.4  && opModeIsActive()) {
            double lx = dx/Math.abs(dx)*Math.min(0.4, Math.abs(dx/2000.0));
            drive.setWeightedDrivePower(new Pose2d(lx,0,0));
            drive.updatePoseEstimate();
            poseEstimate = drive.getPoseEstimate();
            dx = targetPose.getX()-poseEstimate.getX();
        }
    }

    private void conduY(double x) {

        drive.setPoseEstimate(new Pose2d(0,0,0));
        Pose2d poseEstimate = drive.getPoseEstimate();
        Pose2d targetPose = new Pose2d(0,x,0);
        double dx = targetPose.getX()-poseEstimate.getX();
        while (Math.abs(dx)>0.4 && opModeIsActive()) {
            double lx = dx/Math.abs(dx)*Math.min(0.4, Math.abs(dx/2000.0));

            drive.setWeightedDrivePower(new Pose2d(0,lx,0));
            drive.updatePoseEstimate();
            poseEstimate = drive.getPoseEstimate();
            dx = targetPose.getX()-poseEstimate.getX();
        }
    }



    @Override
    public void runOpMode() {
        drive = new Mecanum(hardwareMap);

        arm = new Arm(hardwareMap);

        claw = new Claw(hardwareMap);

        waitForStart();

        if (opModeIsActive() ) {
            conduX(100);
            conduY(100);

            claw.autonomous();
            stop();
            //CUM NAIBA PUN PIXELI IN BACKSTAGE
        }

    }
}

