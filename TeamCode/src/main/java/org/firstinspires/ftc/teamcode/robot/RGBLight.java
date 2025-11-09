
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
    public boolean updateSelf = true;
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
        if (robot.drivetrain.getState() == Drivetrain.State.TELE_SLOW_DRIVE && robot.drivetrain.hasParkSlowDriveScale()) {
            if (robot.drivetrain.inPreciseParkTolerance())
                light.setPosition(params.green);
            else if (robot.drivetrain.inBasicParkTolerance())
                light.setPosition(params.yellow);
            else
                light.setPosition(params.off);
        }
        else if (robot.intake.getState() == Intake.State.FEED_SHOOTER_PRECISE) {
            if (robot.intake.getOfficialNumBalls() == 3)
                light.setPosition(params.white);
            else if (robot.intake.getOfficialNumBalls() == 2)
                light.setPosition(params.blue);
            else if (robot.intake.getOfficialNumBalls() == 1)
                light.setPosition(params.red);
        }
        else if (updateSelf) {
            if (robot.shooter.getState() == Shooter.State.TRACK_PASSIVE_SPEED)
                light.setPosition(params.off);
            else {
                if (robot.shooter.getZone() == Shooter.Zone.NEAR) {
                    if (robot.shooter.getAvgMotorSpeed() > Shooter.nearParams.maxSpeed)
                        light.setPosition(params.blue);
                    else if (robot.shooter.getAvgMotorSpeed() > Shooter.nearParams.minSpeed)
                        light.setPosition(params.green);
                    else if (robot.shooter.getAvgMotorSpeed() > Shooter.nearParams.extraCloseMinSpeed)
                        light.setPosition(params.yellow);
                    else
                        light.setPosition(params.red);
                }
                else {
                    if (robot.shooter.getAvgMotorSpeed() > Shooter.farParams.maxSpeed)
                        light.setPosition(params.blue);
                    else if (robot.shooter.getAvgMotorSpeed() > Shooter.farParams.minSpeed)
                        light.setPosition(params.green);
                    else
                        light.setPosition(params.red);
                }
            }
        }
    }

    public void setColor(double color) {
        updateSelf = false;
        light.setPosition(color);
    }
    public void startUpdatingSelfAgain() {
        updateSelf = true;
    }

    @Override
    public void printInfo() {
    }
}
