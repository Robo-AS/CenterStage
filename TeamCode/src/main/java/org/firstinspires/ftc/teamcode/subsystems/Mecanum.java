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

public class Mecanum extends SampleMecanumDrive {


    public Mecanum(){
        super();

        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
    }

    enum Mode{
        NORMAL,
        AUTO,
    }

    Pose2d blueBoard = new Pose2d(0, 0, 0);
    Pose2d redBoard = new Pose2d(0, 0, 0);



    private Mode mode = Mode.NORMAL;

    public void teleOp(Gamepad gamepad1, Telemetry telemetry){

        update();

        Pose2d poseEstimate = getPoseEstimate();

        telemetry.addData("mode", mode);
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());
        

        switch (mode){
            case NORMAL:
                Pose2d driveDirection = new Pose2d(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x
                 );

                setWeightedDrivePower(driveDirection);

                if(gamepad1.a){
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
                    if(gamepad1.x){
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