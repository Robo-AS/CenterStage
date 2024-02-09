package org.firstinspires.ftc.teamcode.programs.utils;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

@Config
public final class AutonomousConstants {

    public static class Coordinates {
        public double y;
        public double x;
        public double heading;

        public Coordinates(double x, double y, double heading) {
            this.x = x;
            this.y = y;
            this.heading = heading;
        }
    }

    public static Coordinates StartPoseLeftRed = new Coordinates(-35, -67 + 15/2, 270); //
    public static Coordinates StartPoseRightRed = new Coordinates(12, -67 + 15/2, 270);
    public static Coordinates StartPoseLeftBlue = new Coordinates(12, 67 - 15/2, 90);
    public static Coordinates StartPoseRightBlue = new Coordinates(-35, 67 - 15/2, 90);

//    public static Coordinates PixelLeft = new Coordinates(-4.5, 29, 130);
    public static Coordinates PixelLeft = new Coordinates(-9, 23, 130); //spline to pixel

    public static Coordinates PixelMiddle = new Coordinates(0, 28, 90); //spline to pixel
    public static Coordinates PixelRight = new Coordinates(6, 25, 40);
    public static Coordinates PixelForward = new Coordinates(0, 10, 0);

    public static Coordinates Back = new Coordinates(2,-50,0);
    public static Coordinates FarBack = new Coordinates(-5, -58, 90);

    public static Coordinates Park = new Coordinates(40, -35, 360);

    public static Coordinates CornerPark = new Coordinates(52, -64, 0); //left ;a backdropo
//    public static Coordinates FarCornerPark = new Coordinates(55, -12, 180);
public static Coordinates FarCornerPark = new Coordinates(45, -30, 0);  //right la backdrop

    public static Coordinates BackboardMiddle = new Coordinates(40, -40, 0);
    public static Coordinates BackboardRight = new Coordinates(40, -31, 0);
    public static Coordinates BackboardLeft = new Coordinates(40, -47, 0);

    public static Pose2d coordinatesConvert(Coordinates coord) {
        return new Pose2d(coord.x, coord.y, Math.toRadians(coord.heading));
    }


//    public static int SLEEP = 11500;
    public static int SLEEP = 1500;
}
