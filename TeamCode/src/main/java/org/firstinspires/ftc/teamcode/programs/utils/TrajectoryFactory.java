package org.firstinspires.ftc.teamcode.programs.utils;

import static org.firstinspires.ftc.teamcode.programs.utils.AutonomousConstants.PixelForward;
import static org.firstinspires.ftc.teamcode.programs.utils.AutonomousConstants.coordinatesConvert;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.programs.subsystems.Arm;

import org.firstinspires.ftc.teamcode.programs.detection.GameElementDetection;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

import java.util.ArrayList;
import java.util.List;

public class TrajectoryFactory {
    public static List<TrajectorySequence> createTrajectory(SampleMecanumDrive drive, GameElementDetection.Position position, Telemetry telemetry, boolean isBlue, boolean isClose) {
        AutonomousConstants.Coordinates pixelCoordinates;
        Pose2d parkPose = coordinatesConvert(AutonomousConstants.Park);
        Pose2d cornerPose;
        Pose2d backboardPose;
        Pose2d backPose;
        Pose2d startPose;

        List<TrajectorySequence> trajectories = new ArrayList<TrajectorySequence>();

        int multiplier = isBlue ? -1 : 1;
        int backBoardOffset;

        if (isBlue) {
            backBoardOffset = 7;
            if (!isClose) startPose = coordinatesConvert(AutonomousConstants.StartPoseRightBlue);
            else startPose = coordinatesConvert(AutonomousConstants.StartPoseLeftBlue);
        } else {
            backBoardOffset = 0;
            if (isClose) startPose = coordinatesConvert(AutonomousConstants.StartPoseRightRed);
            else startPose = coordinatesConvert(AutonomousConstants.StartPoseLeftRed);
        }

        if (position == GameElementDetection.Position.RIGHT) {
            pixelCoordinates = AutonomousConstants.PixelRight;
            pixelCoordinates.heading = 40;
            backboardPose = coordinatesConvert(AutonomousConstants.BackboardRight);
            if (!isBlue)
                backboardPose = coordinatesConvert(AutonomousConstants.BackboardLeft);
            if (isBlue)
                pixelCoordinates.heading = 130;
        } else if (position == GameElementDetection.Position.MIDDLE) {
            pixelCoordinates = AutonomousConstants.PixelMiddle;
            backboardPose = coordinatesConvert(AutonomousConstants.BackboardMiddle);
        } else {
            pixelCoordinates = AutonomousConstants.PixelLeft;
            pixelCoordinates.heading = 130;
            backboardPose = coordinatesConvert(AutonomousConstants.BackboardLeft);
            if (!isBlue)
                backboardPose = coordinatesConvert(AutonomousConstants.BackboardRight);
            if (isBlue)
                pixelCoordinates.heading = 40;
        }

        if (isClose) {
            backPose = coordinatesConvert(AutonomousConstants.Back);
            cornerPose = coordinatesConvert(AutonomousConstants.CornerPark);
        } else {
            backPose = coordinatesConvert(AutonomousConstants.FarBack);
            cornerPose = coordinatesConvert(AutonomousConstants.FarCornerPark);
        }

        drive.setPoseEstimate(startPose);
        if (isClose && multiplier == -1) {
            trajectories.add(drive.trajectorySequenceBuilder(startPose)
                    .setReversed(true)
                    .lineTo(new Vector2d(startPose.getX(), startPose.getY() + PixelForward.y * multiplier))
                    .splineTo(new Vector2d(startPose.getX() + pixelCoordinates.x * multiplier, startPose.getY() + pixelCoordinates.y * multiplier), Math.toRadians(pixelCoordinates.heading * multiplier))
                    .setReversed(false)
                    .build());

            telemetry.addData("angle: ", drive.getPoseEstimate().getHeading());
            telemetry.addData("angle: ", drive.getPoseEstimate().getX());
            telemetry.addData("angle: ", drive.getPoseEstimate().getY());
            telemetry.update();

            trajectories.add(drive.trajectorySequenceBuilder(new Pose2d(startPose.getX() + pixelCoordinates.x * multiplier, startPose.getY() + pixelCoordinates.y * multiplier, drive.getPoseEstimate().getHeading() ))
                    .setReversed(false)
//                    .lineToConstantHeading(new Vector2d(startPose.getX(), backPose.getY() * multiplier))
                    .lineToLinearHeading(new Pose2d(startPose.getX() + 10, backPose.getY() * multiplier, Math.toRadians(90)))
                    .setReversed(false)
                    .lineToLinearHeading(new Pose2d(parkPose.getX(), parkPose.getY() * multiplier, parkPose.getHeading()))
                    .lineTo(new Vector2d(backboardPose.getX(), (backboardPose.getY() - 0.5) * multiplier))
                    .build());

            trajectories.add(drive.trajectorySequenceBuilder(new Pose2d(backboardPose.getX(), backboardPose.getY() * multiplier, parkPose.getHeading()))
                    .lineTo(new Vector2d(backboardPose.getX() - 2, cornerPose.getY() * multiplier))
                    .lineTo(new Vector2d(cornerPose.getX(), cornerPose.getY() * multiplier))
                    .build());
        }


        else if(isClose && multiplier == 1){
            trajectories.add(drive.trajectorySequenceBuilder(startPose)
                    .setReversed(true)
                    .lineTo(new Vector2d(startPose.getX(), startPose.getY() + PixelForward.y * multiplier))
                    .splineTo(new Vector2d(startPose.getX() + pixelCoordinates.x * multiplier, startPose.getY() + pixelCoordinates.y * multiplier), Math.toRadians(pixelCoordinates.heading * multiplier))
                    .setReversed(false)
                    .build());

            telemetry.addData("angle: ", drive.getPoseEstimate().getHeading());
            telemetry.addData("angle: ", drive.getPoseEstimate().getX());
            telemetry.addData("angle: ", drive.getPoseEstimate().getY());
            telemetry.update();

            trajectories.add(drive.trajectorySequenceBuilder(new Pose2d(startPose.getX() + pixelCoordinates.x * multiplier, startPose.getY() + pixelCoordinates.y * multiplier, drive.getPoseEstimate().getHeading() ))
                    .setReversed(false)
//                    .lineToConstantHeading(new Vector2d(startPose.getX(), backPose.getY() * multiplier))
                    //.lineToLinearHeading(new Pose2d(startPose.getX() + 10, backPose.getY() * multiplier, Math.toRadians(90)))
                    .lineTo(new Vector2d(startPose.getX() + 10, backPose.getY() * multiplier))
                    .setReversed(false)
                    .lineToLinearHeading(new Pose2d(parkPose.getX(), parkPose.getY() * multiplier, parkPose.getHeading()))
                    .lineTo(new Vector2d(backboardPose.getX(), (backboardPose.getY() - 0.5) * multiplier))
                    .build());

            trajectories.add(drive.trajectorySequenceBuilder(new Pose2d(backboardPose.getX(), backboardPose.getY() * multiplier, parkPose.getHeading()))
                    .lineTo(new Vector2d(backboardPose.getX() - 2, cornerPose.getY() * multiplier))
                    .lineTo(new Vector2d(cornerPose.getX(), cornerPose.getY() * multiplier))
                    .build());
        }





        else if(!isClose && multiplier == -1) {
            trajectories.add(drive.trajectorySequenceBuilder(startPose)
                    .setReversed(true)
                    .lineTo(new Vector2d(startPose.getX(), startPose.getY() + PixelForward.y * multiplier))
                    .splineTo(new Vector2d(startPose.getX() + pixelCoordinates.x * multiplier, startPose.getY() + pixelCoordinates.y * multiplier), Math.toRadians(pixelCoordinates.heading * multiplier))
                    .setReversed(false)
                    .splineTo(new Vector2d(startPose.getX() + backPose.getX(), backPose.getY() * multiplier), Math.toRadians(backPose.getHeading() * multiplier))
                    .setReversed(true)
                    .turn(Math.toRadians(180 * multiplier))
                    .lineTo(new Vector2d((parkPose.getX()) - 35, (backPose.getY() + 2) * multiplier))
                    .build());

            trajectories.add(drive.trajectorySequenceBuilder(
                            new Pose2d(parkPose.getX() - 35, (backPose.getY() + 2) * multiplier, parkPose.getHeading()))
                    .setReversed(true)
                    .lineTo(new Vector2d(backboardPose.getX() + 0.5, (backboardPose.getY() + 2) * multiplier + backBoardOffset))
                    .build());

            trajectories.add(drive.trajectorySequenceBuilder(
                            new Pose2d(backboardPose.getX() + 0.5, backboardPose.getY() * multiplier, parkPose.getHeading()))
                    .setReversed(true)
                    .lineTo(new Vector2d(backboardPose.getX() - 3, backboardPose.getY() * multiplier))
                    .lineTo(new Vector2d(backboardPose.getX(), cornerPose.getY() * +multiplier))
                    .build());
        }

        return trajectories;
    }


}