package org.firstinspires.ftc.teamcode.utils.pinpoint;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.utils.math.HeadingCorrect;
import org.firstinspires.ftc.teamcode.utils.math.MathUtils;

import java.util.Arrays;
import java.util.Objects;

@Config
public final class PinpointLocalizer implements Localizer {
    public static class SetupParams {
        // old values
//        public double parYTicks = -141.532;
//        public double perpXTicks = 17.667;
        public double parYTicks = -141.532;
        public double perpXTicks = 1.27;
        public GoBildaPinpointDriver.EncoderDirection initialParDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD, initialPerpDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
    }
    public static class PosePredictParams {
        public double startingDtEstimation = 0.1;
        public int numPrevDeltaTimesToTrack = 10;
        public double[] dtWeights = {0.3, 0.2, 0.15, 0.125, 0.075, 0.05, 0.03, 0.025, 0.025, 0.02 }; // MUST add up to 1
        public int numPrevVelocitiesToTrack = 4;
        public double[] accelerationWeights = { 0.6, 0.3, 0.1 }; // MUST add up to 1
    }

    public static SetupParams setupParams = new SetupParams();
    public static PosePredictParams posePredictParams = new PosePredictParams();

    public final GoBildaPinpointDriver driver;

    private Pose2d txWorldPinpoint;
    private Pose2d txPinpointRobot = new Pose2d(0, 0, 0);
    private final Telemetry telemetry;
    // previousVelocities[0] = most recent
    // previousAccelerations[0] is most recent, calculated with previousVelocities[0] and previousVelocities[1]
    private final double[][] previousVelocities, previousAccelerations;
    private final double[] previousDeltaTimes;
    private double lastUpdateTimeMs;
    private int framesRunning;

    public PinpointLocalizer(HardwareMap hardwareMap, Pose2d initialPose, Telemetry telemetry) {
        this.telemetry = telemetry;

        // TODO: make sure your config has a Pinpoint device with this name
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        driver = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        driver.setEncoderResolution(20, DistanceUnit.MM); //1.0 / mmPerTick (FIX VALUE)
        driver.setOffsets(
                DistanceUnit.MM.fromMm(setupParams.parYTicks),
                DistanceUnit.MM.fromMm(setupParams.perpXTicks),
                DistanceUnit.MM
        );

        driver.setEncoderDirections(setupParams.initialParDirection, setupParams.initialPerpDirection);

        driver.resetPosAndIMU();

        txWorldPinpoint = initialPose;

        previousDeltaTimes = new double[posePredictParams.numPrevDeltaTimesToTrack];
        lastUpdateTimeMs = 0;
        previousVelocities = new double[posePredictParams.numPrevVelocitiesToTrack][4]; // time, x, y, and heading velocity
        previousAccelerations = new double[posePredictParams.numPrevVelocitiesToTrack - 1][3]; // x, y, and heading acceleration

        if (posePredictParams.accelerationWeights.length != posePredictParams.numPrevVelocitiesToTrack - 1)
            throw new IllegalArgumentException(("PosePredictParams has " + posePredictParams.accelerationWeights.length + " accelerationWeights but also specifies " + posePredictParams.numPrevVelocitiesToTrack + " velocities to track. These do not match"));
        double sumOfAccelWeights = Arrays.stream(posePredictParams.accelerationWeights).sum();
        if (Math.abs(sumOfAccelWeights - 1) > 0.00001)
            throw new IllegalArgumentException(("the acceleration weights in PosePredictParams do not add up to 1. They add up to " + sumOfAccelWeights));

        if (posePredictParams.dtWeights.length != posePredictParams.numPrevDeltaTimesToTrack)
            throw new IllegalArgumentException(("PosePredictParams has " + posePredictParams.dtWeights.length + " dtWeights but also specifies " + posePredictParams.numPrevDeltaTimesToTrack + " delta times to track. These do not match"));
        double sumOfDtWeights = Arrays.stream(posePredictParams.dtWeights).sum();
        if (Math.abs(sumOfDtWeights - 1) > 0.00001)
            throw new IllegalArgumentException(("the dt weights in PosePredictParams do not add up to 1. They add up to " + sumOfDtWeights));

    }

    public void resetPoseTo(Pose2d pose, long sleepTime) {
        txPinpointRobot = new Pose2d(0, 0, 0);
        driver.resetPosAndIMU();
        txWorldPinpoint = pose;
        try {
            Thread.sleep(sleepTime);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }
    @Override
    public void setPose(Pose2d pose) {
        txWorldPinpoint = pose.times(txPinpointRobot.inverse());
    }

    @Override
    public Pose2d pose() {
        return txWorldPinpoint.times(txPinpointRobot);
    }

    @Override
    public PoseVelocity2d update() {
        driver.update();
        if (Objects.requireNonNull(driver.getDeviceStatus()) == GoBildaPinpointDriver.DeviceStatus.READY) {
            framesRunning++;
            txPinpointRobot = new Pose2d(driver.getPosX(DistanceUnit.INCH), driver.getPosY(DistanceUnit.INCH), driver.getHeading(UnnormalizedAngleUnit.RADIANS));
            double velX = driver.getVelX(DistanceUnit.INCH), velY = driver.getVelY(DistanceUnit.INCH), velHeadingRad = driver.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);
            Vector2d worldVelocity = new Vector2d(velX, velY);
            Vector2d robotVelocity = Rotation2d.fromDouble(-txPinpointRobot.heading.log()).times(worldVelocity);

            updateDeltaTimes();
            updatePreviousVelocitiesAndAccelerations(velX, velY, velHeadingRad);

            return new PoseVelocity2d(robotVelocity, driver.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS));
        }
        return new PoseVelocity2d(new Vector2d(0, 0), 0);
    }

    public void printInfo() {
        telemetry.addLine("PINPOINT");
        telemetry.addData("position (in)", MathUtils.format2(pose().position.x) + ", " + MathUtils.format2(pose().position.y));
        telemetry.addData("heading (deg)", MathUtils.format2(Math.toDegrees(pose().heading.toDouble())));
    }

    private void updateDeltaTimes() {
        for (int i=previousDeltaTimes.length - 1; i>0; i--)
            previousDeltaTimes[i] = previousDeltaTimes[i - 1];

        double currentTime = System.currentTimeMillis();
        previousDeltaTimes[0] = currentTime - lastUpdateTimeMs;
        lastUpdateTimeMs = currentTime;
    }
    private double getWeightedDeltaTime() {
        if (framesRunning < posePredictParams.numPrevDeltaTimesToTrack)
            return posePredictParams.startingDtEstimation;

        double dt = 0;
        for (int i=0; i<previousDeltaTimes.length; i++)
            dt += previousDeltaTimes[i] * posePredictParams.dtWeights[i];
        return dt;
    }
    public Pose2d getNextPoseSimple() {
        double xVel = driver.getVelX(DistanceUnit.INCH);
        double yVel = driver.getVelY(DistanceUnit.INCH);
        double headingVelRad = driver.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);
        double correctedHeadingVelRad = HeadingCorrect.correctHeadingErrorRad(headingVelRad);

        return new Pose2d(pose().position.x + xVel, pose().position.y + yVel, pose().heading.toDouble() + correctedHeadingVelRad);
    }
    public Pose2d getNextPoseMultipleFrames() {
        // find weighted average of acceleration
        double[] averageAcceleration = new double[3];
        for (int i=0; i<previousAccelerations.length; i++)
            for (int j = 0; j < previousAccelerations[0].length; j++)
                averageAcceleration[j] += previousAccelerations[i][j] * posePredictParams.accelerationWeights[i];

        // predict the velocity of next frame
        // new velocity = old velocity + acceleration (velocities and acceleration are already in the correct units)
        double[] nextVelocity = previousVelocities[0];
        for (int i=1; i<4; i++)
            nextVelocity[i] += averageAcceleration[i - 1];

        // predict the position of next frame
        // new position = old position + velocity
        return new Pose2d(pose().position.x + nextVelocity[0], pose().position.y + nextVelocity[1], pose().heading.toDouble() + nextVelocity[2]);
    }
    private void updatePreviousVelocitiesAndAccelerations(double xVel, double yVel, double headingRadVel) {
        double weightedDeltaTime = getWeightedDeltaTime();

        // push velocities back
        for (int i=previousVelocities.length - 1; i>0; i--)
            previousVelocities[i] = previousVelocities[i - 1];

        // add most recent velocity (scaled by dt)
        // velocity is already in in/sec, want to convert to distance traveled between this frame and last frame
        previousVelocities[0][0] = xVel * weightedDeltaTime;
        previousVelocities[0][1] = yVel * weightedDeltaTime;
        previousVelocities[0][2] = headingRadVel * weightedDeltaTime;

        // push acceleration back
        for (int i=previousAccelerations.length - 1; i>0; i--)
            previousAccelerations[i] = previousAccelerations[i - 1];

        // update acceleration
        // acceleration = current velocity - old velocity
        // not dividing by time b/c velocity is already in the desired "time" unit - change from last frame to this frame
        // so this acceleration actually represents the change in velocity from last frame to this frame
        for (int j=0; j<3; j++)
            previousAccelerations[0][j] = (previousVelocities[0][j] - previousVelocities[1][j]);
    }
}