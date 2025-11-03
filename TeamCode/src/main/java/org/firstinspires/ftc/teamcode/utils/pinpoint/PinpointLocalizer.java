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
import org.firstinspires.ftc.teamcode.utils.misc.MathUtils;

import java.util.Objects;

@Config
public final class PinpointLocalizer implements Localizer {
    public static class Params {
        // old values
//        public double parYTicks = -141.532;
//        public double perpXTicks = 17.667;
        public double parYTicks = -141.532;
        public double perpXTicks = 1.27;
        public GoBildaPinpointDriver.EncoderDirection initialParDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD, initialPerpDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
    }

    public static Params PARAMS = new Params();

    public final GoBildaPinpointDriver driver;

    private Pose2d txWorldPinpoint;
    private Pose2d txPinpointRobot = new Pose2d(0, 0, 0);
    private final Telemetry telemetry;

    public PinpointLocalizer(HardwareMap hardwareMap, Pose2d initialPose, Telemetry telemetry) {
        this.telemetry = telemetry;

        // TODO: make sure your config has a Pinpoint device with this name
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        driver = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        driver.setEncoderResolution(20, DistanceUnit.MM); //1.0 / mmPerTick (FIX VALUE)
        driver.setOffsets(
                DistanceUnit.MM.fromMm(PARAMS.parYTicks),
                DistanceUnit.MM.fromMm(PARAMS.perpXTicks),
                DistanceUnit.MM
        );

        driver.setEncoderDirections(PARAMS.initialParDirection, PARAMS.initialPerpDirection);

        driver.resetPosAndIMU();

        txWorldPinpoint = initialPose;
    }

    public void setInitialPose(Pose2d pose) {
        txWorldPinpoint = pose;
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
            txPinpointRobot = new Pose2d(driver.getPosX(DistanceUnit.INCH), driver.getPosY(DistanceUnit.INCH), driver.getHeading(UnnormalizedAngleUnit.RADIANS));
            Vector2d worldVelocity = new Vector2d(driver.getVelX(DistanceUnit.INCH), driver.getVelY(DistanceUnit.INCH));
            Vector2d robotVelocity = Rotation2d.fromDouble(-txPinpointRobot.heading.log()).times(worldVelocity);

            return new PoseVelocity2d(robotVelocity, driver.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS));
        }
        return new PoseVelocity2d(new Vector2d(0, 0), 0);
    }

    public void printInfo() {
        telemetry.addLine("PINPOINT");
        telemetry.addData("position (in)", MathUtils.format2(pose().position.x) + ", " + MathUtils.format2(pose().position.y));
        telemetry.addData("heading (deg)", MathUtils.format2(Math.toDegrees(pose().heading.toDouble())));
    }
}