
package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.stateManagement.Subsystem;

@Config
public class RGBLight extends Subsystem {
    public static class Params {
        public double off = 0.01;
        public double red = 0.11, yellow = 0.21, green = 0.45, blue = 0.61, purple = 0.84, white = 0.91;
    }
    public static Params params = new Params();
    private ServoImplEx light;
    public RGBLight(Hardware hardware, Telemetry telemetry) {
        super(hardware, telemetry);
    }

    @Override
    public void declareHardware() {
        light = hardware.getRGBLight();
        light.setPosition(params.white);
    }

    @Override
    public void updateState() {
        if (robot.intake.getState() == Intake.State.FEED_SHOOTER_PRECISE) {
            if (robot.intake.getOfficialNumBalls() == 3)
                light.setPosition(params.white);
            else if (robot.intake.getOfficialNumBalls() == 2)
                light.setPosition(params.blue);
            else if (robot.intake.getOfficialNumBalls() == 1)
                light.setPosition(params.red);
        }
        else {
            int readiness = 0;

            if (robot.drivetrain.headingWithinShootingTolerance())
                readiness++;
            if (robot.shooter.canShoot())
                readiness++;

            if (readiness == 0)
                light.setPosition(params.red);
            else if (readiness == 1)
                light.setPosition(params.yellow);
            else
                light.setPosition(params.green);
        }

    }

    @Override
    public void printInfo() {
    }
}
