package org.firstinspires.ftc.teamcode.utils.pidDrive;

import com.acmerobotics.dashboard.config.Config;

@Config
public class DriveParams {
    private static final double noMaxTime = -1;

    public static double defaultSpeedKp = 0.05, defaultSpeedKi = 0.001, defaultSpeedKd = 0.004;
    public static double defaultHeadingKp = 0.6, defaultHeadingKi = 0.01, defaultHeadingKd = 0;
    public static double defaultLateralWeight = 1.2, defaultAxialWeight = 1; // weight the drive powers to correct for differences in driving

    public double lateralWeight, axialWeight;
    public double minSpeed, maxSpeed;
    public double minHeadingSpeed, maxHeadingSpeed;
    public double slowDownPercent; // if this equals 1, then the drivetrain will completely stop at this waypoint; if this equals 0, this waypoint will have no influence on slowing down the drivetrain as it approaches this point
    public boolean passPosition; // the robot only needs to pass its target position, not fall within tolerance of it
    public double maxTime;

    public double speedKp, speedKi, speedKd, headingKp, headingKi, headingKd;

    public DriveParams() {
        this(defaultSpeedKp, defaultSpeedKi, defaultSpeedKd, defaultHeadingKp, defaultHeadingKi, defaultHeadingKd);
    }
    public DriveParams(double speedKp, double speedKi, double speedKd, double headingKp, double headingKi, double headingKd) {
        this.speedKp = speedKp;
        this.speedKi = speedKi;
        this.speedKd = speedKd;
        this.headingKp = headingKp;
        this.headingKi = headingKi;
        this.headingKd = headingKd;

        maxTime = Double.MAX_VALUE;
        minSpeed = 0;
        maxSpeed = Double.MAX_VALUE;
        minHeadingSpeed = 0;
        maxHeadingSpeed = Double.MAX_VALUE;
        lateralWeight = defaultLateralWeight;
        axialWeight = defaultAxialWeight;
        passPosition = false;
    }

    public void setWeights(double lateral, double axial) {
        lateralWeight = lateral;
        axialWeight = axial;
    }
    public void setLinearSpeedConstraints(double minSpeed, double maxSpeed) {
        this.minSpeed = minSpeed;
        this.maxSpeed = maxSpeed;
    }
    public void setHeadingSpeedConstraints(double minHeadingSpeed, double maxHeadingSpeed) {
        this.minHeadingSpeed = minHeadingSpeed;
        this.maxHeadingSpeed = maxHeadingSpeed;
    }
    public boolean hasMaxTime() {
        return maxTime != noMaxTime;
    }

}
