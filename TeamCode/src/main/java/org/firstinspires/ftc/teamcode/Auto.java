/* Created by Phil Malone. 2023.
    This class illustrates my simplified Odometry Strategy.
    It implements basic straight line motions but with heading and drift controls to limit drift.
    See the readme for a link to a video tutorial explaining the operation and limitations of the code.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.ODOMETRY_AUTO.SimplifiedOdometryRobot;
import org.firstinspires.ftc.teamcode.subsystems.Arm;

/*
 * This OpMode illustrates an autonomous opmode using simple Odometry
 * All robot functions are performed by an external "Robot" class that manages all hardware interactions.
 * Pure Drive or Strafe motions are maintained using two Odometry Wheels.
 * The IMU gyro is used to stabilize the heading during all motions
 */

@Autonomous(name="Sample Autonomous", group = "Mr. Phil")
public class Auto extends LinearOpMode {
    // get an instance of the "Robot" class.
    SimplifiedOdometryRobot mecanum;

    Arm arm;





    @Override public void runOpMode() {

        SimplifiedOdometryRobot mecanum = new SimplifiedOdometryRobot(this);

        arm = new Arm(hardwareMap);
        // Initialize the robot hardware & Turn on telemetry
        mecanum.initialize(true);

        // Wait for driver to press start
        telemetry.addData(">", "Touch Play to run Auto");
        telemetry.update();

        waitForStart();
        mecanum.resetHeading();  // Reset heading to set a baseline for Auto

        // Run Auto if stop was not pressed.
        if (opModeIsActive())
        {
            // Note, this example takes more than 30 seconds to execute, so turn OFF the auto timer.

            // Drive a large rectangle, turning at each corner
            mecanum.drive(  84, 0.60, 0.25);
            mecanum.turnTo(90, 0.45, 0.5);
            mecanum.drive(  72, 0.60, 0.25);
            mecanum.turnTo(180, 0.45, 0.5);
            mecanum.drive(  84, 0.60, 0.25);
            mecanum.turnTo(270, 0.45, 0.5);
            mecanum.drive(  72, 0.60, 0.25);
            mecanum.turnTo(0, 0.45, 0.5);

            sleep(500);

            // Drive the path again without turning.
            mecanum.drive(  84, 0.60, 0.15);
            mecanum.strafe( 72, 0.60, 0.15);
            mecanum.drive( -84, 0.60, 0.15);
            mecanum.strafe(-72, 0.60, 0.15);




        }
    }
}
