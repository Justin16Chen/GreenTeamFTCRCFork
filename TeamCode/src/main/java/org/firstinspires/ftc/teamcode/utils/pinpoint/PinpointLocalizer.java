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

import java.util.ArrayList;
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
        public int numPrevVelocitiesToTrack = 4;
        public double maxLinearSpeedPerSecond = 10, maxHeadingDegSpeedPerSecond = 20;
        public boolean clampSpeeds = false;
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
    public final ArrayList<OdoInfo> previousVelocities, previousAccelerations;
    private double lastUpdateTimeMs;
    public Pose2d lastPose;
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
        lastPose = initialPose;

        lastUpdateTimeMs = 0;
        previousVelocities = new ArrayList<>();  // each entry contains x, y, and heading velocity
        previousAccelerations = new ArrayList<>(); // each entry contains x, y, and heading acceleration

        if (posePredictParams.accelerationWeights.length != posePredictParams.numPrevVelocitiesToTrack - 1)
            throw new IllegalArgumentException(("PosePredictParams has " + posePredictParams.accelerationWeights.length + " accelerationWeights but also specifies " + posePredictParams.numPrevVelocitiesToTrack + " velocities to track. These do not match"));
        double sumOfAccelWeights = Arrays.stream(posePredictParams.accelerationWeights).sum();
        if (Math.abs(sumOfAccelWeights - 1) > 0.00001)
            throw new IllegalArgumentException(("the acceleration weights in PosePredictParams do not add up to 1. They add up to " + sumOfAccelWeights));
    }

    public void resetPoseTo(Pose2d pose, long sleepTime) {
        txPinpointRobot = new Pose2d(0, 0, 0);
        driver.resetPosAndIMU();
        txWorldPinpoint = new Pose2d(pose.position.x, pose.position.y, pose.heading.toDouble());;
        lastPose = new Pose2d(pose.position.x, pose.position.y, pose.heading.toDouble());
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
            lastPose = pose();

            txPinpointRobot = new Pose2d(driver.getPosX(DistanceUnit.INCH), driver.getPosY(DistanceUnit.INCH), driver.getHeading(UnnormalizedAngleUnit.RADIANS));
            double velX = driver.getVelX(DistanceUnit.INCH), velY = driver.getVelY(DistanceUnit.INCH), velHeadingRad = driver.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);
            Vector2d worldVelocity = new Vector2d(velX, velY);
            Vector2d robotVelocity = Rotation2d.fromDouble(-txPinpointRobot.heading.log()).times(worldVelocity);

            updatePreviousVelocitiesAndAccelerations();

            return new PoseVelocity2d(robotVelocity, driver.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS));
        }
        return new PoseVelocity2d(new Vector2d(0, 0), 0);
    }

    public void printInfo() {
        telemetry.addLine("PINPOINT");
        telemetry.addData("position (in)", MathUtils.format2(pose().position.x) + ", " + MathUtils.format2(pose().position.y));
        telemetry.addData("heading (deg)", MathUtils.format2(Math.toDegrees(pose().heading.toDouble())));
    }
    // predicts on most recent velocity
    public Pose2d getNextPoseSimple() {
        return new Pose2d(pose().position.x + previousVelocities.get(0).x, pose().position.y + previousVelocities.get(0).y, pose().heading.toDouble() + previousVelocities.get(0).headingRad);
    }
    // predicts based on acceleration
    public Pose2d getNextPoseAdvanced() {
        // find weighted average of acceleration
        OdoInfo weightedAcceleration = new OdoInfo();
        for (int i=0; i<previousAccelerations.size(); i++) {
            weightedAcceleration.x += previousAccelerations.get(i).x * posePredictParams.accelerationWeights[i];
            weightedAcceleration.y += previousAccelerations.get(i).y * posePredictParams.accelerationWeights[i];
            weightedAcceleration.headingRad += previousAccelerations.get(i).headingRad * posePredictParams.accelerationWeights[i];
        }

        // predict the velocity of next frame
        // new velocity = old velocity + acceleration (velocities and acceleration are already in the correct units)
        OdoInfo nextVelocity = previousVelocities.get(0);
        nextVelocity.x += weightedAcceleration.x;
        nextVelocity.y += weightedAcceleration.y;
        nextVelocity.headingRad += weightedAcceleration.headingRad;

        // predict the position of next frame
        // new position = old position + velocity
        return new Pose2d(pose().position.x + nextVelocity.x, pose().position.y + nextVelocity.y, pose().heading.toDouble() + nextVelocity.headingRad);
    }
    private void updatePreviousVelocitiesAndAccelerations() {
        // remove oldest velocity
        if (previousVelocities.size() > posePredictParams.numPrevVelocitiesToTrack)
            previousVelocities.remove(previousVelocities.size() - 1);

        // add most recent velocity (change in position between this frame and last frame)
        // technically units would be inches/deltaTime
        Pose2d curPose = pose();
        double dx = curPose.position.x - lastPose.position.x;
        double dy = curPose.position.y - lastPose.position.y;
        double dh = HeadingCorrect.correctHeadingErrorRad(curPose.heading.toDouble() - lastPose.heading.toDouble());

        // clamp linear and heading velocity
        if (posePredictParams.clampSpeeds) {
            double dt = (System.currentTimeMillis() - lastUpdateTimeMs) / 1000.0;
            double linearSpeedPerSecond = Math.sqrt(dx * dx + dy * dy) / dt;
            if (linearSpeedPerSecond > posePredictParams.maxLinearSpeedPerSecond) {
                dx *= posePredictParams.maxLinearSpeedPerSecond / linearSpeedPerSecond;
                dy *= posePredictParams.maxLinearSpeedPerSecond / linearSpeedPerSecond;
            }
            double headingSpeedPerSecond = dh / dt;
            if (Math.abs(headingSpeedPerSecond) > Math.toRadians(posePredictParams.maxHeadingDegSpeedPerSecond))
                dh = Math.signum(dh) * Math.toRadians(posePredictParams.maxHeadingDegSpeedPerSecond) * dt;
        }
        previousVelocities.add(0, new OdoInfo(dx, dy, dh));

        // remove oldest acceleration
        if (previousAccelerations.size() > posePredictParams.numPrevVelocitiesToTrack - 1)
            previousAccelerations.remove(previousAccelerations.size() - 1);

        // update acceleration
        // acceleration = current velocity - old velocity
        // not dividing by time b/c velocity is already in the desired "time" unit - change from last frame to this frame
        // so this acceleration actually represents the change in velocity from last frame to this frame
        if (previousVelocities.size() > 1)
            previousAccelerations.add(0, new OdoInfo(
                    previousVelocities.get(0).x - previousVelocities.get(1).x,
                    previousVelocities.get(0).y - previousVelocities.get(1).y,
                    previousVelocities.get(0).headingRad - previousVelocities.get(1).headingRad
            ));
        lastUpdateTimeMs = System.currentTimeMillis();
    }
}