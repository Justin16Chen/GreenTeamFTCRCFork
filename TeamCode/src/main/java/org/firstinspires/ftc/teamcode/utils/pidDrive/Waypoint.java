package org.firstinspires.ftc.teamcode.utils.pidDrive;


import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.utils.misc.MathUtils;

public class Waypoint {
    public Pose2d pose;
    public Tolerance tolerance;
    public DriveParams params;
    private double distToNextWaypoint;
   public Waypoint(Pose2d pose, Tolerance tolerance, DriveParams driveParams) {
        this.pose = pose;
        this.tolerance = tolerance;
        this.params = driveParams;
    }
    public double x() {
        return pose.position.x;
    }
    public double y() {
        return pose.position.y;
    }
    public double headingDeg() {
        return Math.toDegrees(pose.heading.toDouble());
    }
    public double headingRad() {
       return pose.heading.toDouble();
    }

    public void setDistToNextWaypoint(Waypoint waypoint) {
        distToNextWaypoint = Math.sqrt(Math.pow(waypoint.x() - x(), 2) + Math.pow(waypoint.y() - y(), 2));
    }
    public double getDistToNextWaypoint() {
        return distToNextWaypoint;
    }


    @Override
    @NonNull
    public String toString() {
       return "x: " + x() + ", y: " + y() + ", heading: " + MathUtils.format2(headingDeg());
    }
}
