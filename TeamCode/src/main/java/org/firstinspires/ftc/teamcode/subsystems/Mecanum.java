package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

public class Mecanum extends SampleMecanumDrive {


    public Mecanum(HardwareMap hardwareMap){
        super(hardwareMap);

        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
    }

    enum Mode{
        NORMAL,
        AUTO,
    }

    Pose2d blueBoard = new Pose2d(0, 0, 0);
    Pose2d redBoard = new Pose2d(0, 0, 0);



    private Mode mode = Mode.NORMAL;

    public void teleOp(GamepadEx gamepad, Telemetry telemetry){

        update();

        Pose2d poseEstimate = getPoseEstimate();

        telemetry.addData("mode", mode);
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());
        

        switch (mode){
            case NORMAL:
                Pose2d driveDirection = new Pose2d(
                -gamepad.getLeftY(),
                -gamepad.getLeftX(),
                -gamepad.getRightX()
                 );

                setWeightedDrivePower(driveDirection);

                if(gamepad.wasJustPressed(GamepadKeys.Button.A)){
                    //go to blue board

                    Trajectory traj = trajectoryBuilder(poseEstimate)
                        .splineToLinearHeading(blueBoard, 0)
                        .build();

                    followTrajectoryAsync(traj);
                    
                    mode = Mode.AUTO;
                }
                break;

                    break;
                case AUTO:
                    if(gamepad.wasJustPressed(GamepadKeys.Button.X)){
                        breakFollowing();
                        mode = Mode.NORMAL;
                    }    

                    if(!isBusy()){
                        mode = Mode.NORMAL;
                    }

                    break;
        }

        

    }


}