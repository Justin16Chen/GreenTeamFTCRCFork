package org.firstinspires.ftc.teamcode.utils.pidDrive;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.robot.Drivetrain;
import org.firstinspires.ftc.teamcode.utils.misc.PIDFController;
import org.firstinspires.ftc.teamcode.utils.pinpoint.Pinpoint;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Set;

public class DrivePath implements Command {
    private final Drivetrain drivetrain;
    private final Pinpoint odo;
    private final ArrayList<Waypoint> waypoints;
    private int curWaypointIndex; // the waypoint the drivetrain is currently trying to go to
    private PIDFController totalDistancePID, waypointDistancePID, headingPID;
    private final ElapsedTime waypointTimer;
    private boolean reachedDestination;

    public DrivePath(Drivetrain drivetrain, Pinpoint odo, Waypoint destination) {
        this.drivetrain = drivetrain;
        this.odo = odo;

        waypoints = new ArrayList<>();
        waypoints.add(destination);
        curWaypointIndex = 0;

        waypointTimer = new ElapsedTime();
        reachedDestination = false;
    }

    public void addWaypoint(Waypoint waypoint) {
        waypoint.setDistToNextWaypoint(waypoints.get(waypoints.size() - 1));
        waypoints.add(waypoints.size() - 1, waypoint);
        if(waypoints.size() >= 3)
            waypoints.get(waypoints.size() - 3).setDistToNextWaypoint(waypoint);
    }
    public Waypoint getWaypoint(int index) {
        return waypoints.get(index);
    }
    private Waypoint getCurWaypoint() {
        return waypoints.get(Math.min(waypoints.size() - 1, curWaypointIndex));
    }
    private DriveParams getCurParams() {
        return getCurWaypoint().params;
    }
    private double getWaypointDistanceToTarget(int waypointIndex) {
        if (waypointIndex == waypoints.size() - 1)
            return 0;
        double dist = 0;
        for (int i=waypointIndex; i<waypoints.size() - 1; i++)
            dist += waypoints.get(i).getDistToNextWaypoint();
        return dist;
    }
    private Vector2d updateTargetDir(double x, double y, double headingRad) {
        // translating target so that drivetrain is around origin
        double xFromRobot = getCurWaypoint().x() - x;
        double yFromRobot = getCurWaypoint().y() - y;
        // rotating target around origin
        double rotatedXFromRobot = xFromRobot * Math.cos(-headingRad) - yFromRobot * Math.sin(-headingRad);
        double rotatedYFromRobot = xFromRobot * Math.sin(-headingRad) + yFromRobot * Math.cos(-headingRad);
        // translating target back to absolute; this returns the direction to the next waypoint IN THE ROBOT'S COORDINATE PLANE
        Vector2d targetDir = new Vector2d(rotatedXFromRobot, rotatedYFromRobot);
        double targetDirMag = Math.sqrt(Math.pow(targetDir.getX(), 2) + Math.pow(targetDir.getY(), 2));
        return targetDir.div(targetDirMag); // normalize
    }

    @Override
    public void initialize() {
        resetToNewWaypoint();
    }

    @Override
    public void execute() {
        Pose2D pose = odo.getPose();
        double x = pose.getX(DistanceUnit.INCH), y = pose.getY(DistanceUnit.INCH), headingRad = pose.getHeading(AngleUnit.RADIANS);

        // finding direction that motor powers should be applied in
        Vector2d targetDir = updateTargetDir(x, y, headingRad);

        // note: error is calculated in field's coordinate plane
        double xWaypointError = Math.abs(pose.getX(DistanceUnit.INCH) - getCurWaypoint().x());
        double yWaypointError = Math.abs(pose.getY(DistanceUnit.INCH) - getCurWaypoint().y());
        double headingWaypointError = Math.abs(headingRad - getCurWaypoint().headingRad());

        // tolerance
        boolean inPositionTolerance = xWaypointError <= getCurWaypoint().tolerance.xTol && yWaypointError <= getCurWaypoint().tolerance.yTol;
        boolean inHeadingTolerance = headingWaypointError <= getCurWaypoint().tolerance.headingRadTol;

        // pass position
//        double dot = -1;
//        if(getCurParams().passPosition) {
//            Vector2d oldPosition = curWaypointIndex == 0 ? startPose.position : waypoints.get(curWaypointIndex - 1).pose.position;
//            Vector2d targetWaypointPosition = getCurWaypoint().pose.position;
//
//            Vector2d relativeWaypoint = targetWaypointPosition.minus(oldPosition);
//            Vector2d relativePositionToWaypoint = pose.position.minus(targetWaypointPosition);
//            dot = relativeWaypoint.x * relativePositionToWaypoint.x + relativeWaypoint.y * relativePositionToWaypoint.y;
//            if (dot > 0)
//                inPositionTolerance = true;
//        }

        boolean inWaypointTolerance = inPositionTolerance && inHeadingTolerance;

        // in tolerance
        if (inWaypointTolerance || (getCurParams().hasMaxTime() && waypointTimer.seconds() >= getCurParams().maxTime)) {
            curWaypointIndex++;
            if (curWaypointIndex >= waypoints.size())
                return;
            else {
                // set new PID targets
                resetToNewWaypoint();

                // recalculate new waypoint errors
                xWaypointError = Math.abs(pose.getX(DistanceUnit.INCH) - getCurWaypoint().x());
                yWaypointError = Math.abs(pose.getY(DistanceUnit.INCH) - getCurWaypoint().y());
            }
        }

        // calculate inputs to speed PIDs
        double waypointDistAway = Math.sqrt(xWaypointError * xWaypointError + yWaypointError * yWaypointError);
        double totalDistanceAway = waypointDistAway + getWaypointDistanceToTarget(curWaypointIndex);

        // calculate interpolated value between speed PIDs
        double a = Math.abs(totalDistancePID.update(totalDistanceAway));
        double b = Math.abs(waypointDistancePID.update(waypointDistAway));
        double t = getCurParams().slowDownPercent;
        double speed = a + (b - a) * t;
        speed = Range.clip(speed, getCurParams().minSpeed, getCurParams().maxSpeed);

        // calculate directional powers
        double lateralPower = targetDir.getX() * speed * getCurParams().lateralWeight;
        double axialPower = targetDir.getY() * speed * getCurParams().axialWeight;
        double headingPower = headingPID.update(headingRad);
        double headingSign = Math.signum(headingPower);
        headingPower = headingSign * Range.clip(Math.abs(headingPower), getCurParams().minHeadingSpeed, getCurParams().maxHeadingSpeed);

        drivetrain.setDrivePowers(lateralPower, axialPower, headingPower);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setMotorPowers(0, 0, 0, 0);
    }
    @Override
    public boolean isFinished() {
        return reachedDestination;
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Collections.emptySet();
    }
    private void resetToNewWaypoint() {
        waypointTimer.reset();

        totalDistancePID = new PIDFController(getCurParams().speedKp, getCurParams().speedKi, getCurParams().speedKd, 0);
        totalDistancePID.reset();
        totalDistancePID.setTarget(0);
        totalDistancePID.setOutputBounds(0, 1);
        waypointDistancePID = new PIDFController(getCurParams().speedKp, getCurParams().speedKi, getCurParams().speedKd, 0);
        waypointDistancePID.reset();
        waypointDistancePID.setTarget(0);
        waypointDistancePID.setOutputBounds(0, 1);

        headingPID = new PIDFController(getCurParams().headingKp, getCurParams().headingKi, getCurParams().headingKd, 0);
        headingPID.reset();
        headingPID.setTarget(getCurWaypoint().headingDeg());
        headingPID.setOutputBounds(-1, 1);
    }
}

