package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

public class Mecanum extends SampleMecanumDrive {

    public static double reduction = 0.5;

    public Mecanum(HardwareMap hardwareMap) {
        super(hardwareMap);
        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    enum Mode {
        NORMAL,
        AUTO,
    }

    Pose2d blueBoard = new Pose2d(0, 0, 0);
    Pose2d redBoard = new Pose2d(0, 0, 0);

    private boolean fieldOriented = false;
    private Mode mode = Mode.NORMAL;

    public void teleop(GamepadEx gamepad, Telemetry telemetry) {

        update();

        Pose2d poseEstimate = getPoseEstimate();

        telemetry.addData("mode", mode);
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());

        switch (mode) {
            case NORMAL:

                //double mult = (1 - gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) * 0.80) *reduction;

                //AM MODIFICAT AICI SA NU MA SCADA GRADUAL MISCAREA
                double mult = 0.5;
                if(gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)>0.5)
                    mult = 0.2;        ;

                double x = gamepad.getLeftY();
                double y = -gamepad.getLeftX();
                double h = -gamepad.getRightX();

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
                        h * mult);



                setWeightedDrivePower(driveDirection);

                /*
                 * if(gamepad.wasJustPressed(GamepadKeys.Button.A)){
                 * //go to blue board
                 * 
                 * Trajectory traj = trajectoryBuilder(poseEstimate)
                 * .splineToLinearHeading(blueBoard, 0)
                 * .build();
                 * 
                 * followTrajectoryAsync(traj);
                 * 
                 * mode = Mode.AUTO;
                 * }
                 */
                break;

            case AUTO:
                if (gamepad.wasJustPressed(GamepadKeys.Button.X)) {
                    breakFollowing();
                    mode = Mode.NORMAL;
                }

                if (!isBusy()) {
                    mode = Mode.NORMAL;
                }

                break;
        }

    }

}