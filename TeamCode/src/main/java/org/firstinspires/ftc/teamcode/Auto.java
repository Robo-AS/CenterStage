package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Mecanum;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.hardware.HardwareMap;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

@Autonomous(name = "Auto")
public class Auto extends LinearOpMode {
    // Drive function with 3 parameters

    private ElapsedTime     runtime = new ElapsedTime();

    Mecanum drive = new Mecanum(hardwareMap);

    private void conduX(double x) {

        drive.setPoseEstimate(new Pose2d(0,0,0));
        Pose2d poseEstimate = drive.getPoseEstimate();
        Pose2d targetPose = new Pose2d(x,0,0);
        double dx = targetPose.getX()-poseEstimate.getX();
        while (Math.abs(dx)>0.4) {
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
        while (Math.abs(dx)>0.4) {
            double lx = dx/Math.abs(dx)*Math.min(0.4, Math.abs(dx/2000.0));

            drive.setWeightedDrivePower(new Pose2d(0,lx,0));
            drive.updatePoseEstimate();
            poseEstimate = drive.getPoseEstimate();
            dx = targetPose.getX()-poseEstimate.getX();
        }
    }



    @Override
    public void runOpMode() {

        waitForStart();

        if (opModeIsActive()) {
            conduX(100);
            conduY(100);
        }

    }
}

