package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

public class Mecanum extends SampleMecanumDrive {

    public static double reduction = 0.5;

    public static double MIN_JOY_VAL= 0.01;
    public Mecanum(HardwareMap hardwareMap) {
        super(hardwareMap);
        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    enum Mode {
        NORMAL,
        AUTO,
    }

    Pose2d targetPose = new Pose2d(0.0,0.0,0.0);

    private boolean fieldOriented = false;
    private Mode mode = Mode.NORMAL;

    public void teleop(GamepadEx gamepad, Telemetry telemetry) {

        update();

        Pose2d poseEstimate = getPoseEstimate();

//        telemetry.addData("mode", mode);
//        telemetry.addData("x", poseEstimate.getX());
//        telemetry.addData("y", poseEstimate.getY());
//        telemetry.addData("heading", poseEstimate.getHeading());

        switch (mode) {
            case NORMAL:

                //double mult = (1 - gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) * 0.80) *reduction;

                //AM MODIFICAT AICI SA NU MA SCADA GRADUAL MISCAREA
                double mult = 0.5;
                double mult_rotation = 0.7;
                if(gamepad.isDown(GamepadKeys.Button.LEFT_BUMPER)){
                    mult = 0.2;
                    mult_rotation = 0.3;//SCAD ASTA SA FAC ROTATIA MAI INCEATA
                }


                double x = -gamepad.getLeftY();
                double y = gamepad.getLeftX()*2;
                double h = gamepad.getRightX()*0.9;


//                if (Math.abs(x) < MIN_JOY_VAL) {x=0;}
//                if (Math.abs(y) < MIN_JOY_VAL) {y=0;}


                if (fieldOriented) {
                    double angle = poseEstimate.getHeading();
                    double new_y = y * Math.cos(angle) + x * Math.sin(angle);
                    double new_x = -y * Math.sin(angle) + x * Math.cos(angle);
                    y = new_y;
                    x = new_x;
                }

                Pose2d driveDirection = new Pose2d(
                        x * mult,
                        y * mult,
                        h * mult_rotation);



                setWeightedDrivePower(driveDirection);

                  //if(gamepad.wasJustPressed(GamepadKeys.Button.A)){
                  //go to blue board
                  //mode = Mode.AUTO;
                  //}

                break;

            case AUTO:

                double dx = targetPose.getX()-poseEstimate.getX();
                double dy = targetPose.getY()-poseEstimate.getY();
                double dh = targetPose.getHeading()-poseEstimate.getHeading();

                double lx = dx/Math.abs(dx)*Math.min(0.4, Math.abs(dx/2000.0));

                double ly = dy/Math.abs(dy)*Math.min(0.4, Math.abs(dy/2000.0));

                double lh = dh/Math.abs(dh)*Math.min(0.2, Math.abs(dh)/2000.0);

                Pose2d newDir = new Pose2d(
                        lx, ly, lh
                );

                setWeightedDrivePower(newDir);

                if (gamepad.wasJustPressed(GamepadKeys.Button.A)) {
                    mode = Mode.NORMAL;
                }

                if (Math.abs(dx)<6 && Math.abs(dy)<6 && Math.abs(dh)<6){
                    mode = Mode.NORMAL;
                }

                break;
        }

    }

}