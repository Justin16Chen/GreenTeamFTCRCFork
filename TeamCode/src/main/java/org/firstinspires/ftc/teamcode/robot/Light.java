package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.stateManagement.Subsystem;

@Config
public class Light extends Subsystem {
    private static double inverseLerp(double a, double b, double x) {
        return (x - a) / (b - a);
    }

    // x = min + (max - min) * t
    // t = (x - min)/(max-min)
    public static class Params {
        public double off = inverseLerp(Hardware.minLightPWM, Hardware.maxLightPWM, 600); // converting PWM of 600 into corresponding decimal position
        public double bright = inverseLerp(Hardware.minLightPWM, Hardware.maxLightPWM, 1000);
        public double dim = inverseLerp(Hardware.minLightPWM, Hardware.maxLightPWM, 750);
        public double flashTime = 0.8;
    }
    public static Params params = new Params();
    private ServoImplEx light;
    private double lightValue;
    public enum State {
        PASSIVE,
        FLASH
    }
    private State state;
    private final ElapsedTime flashTimer;
    private double passiveValue, flashOffValue, flashOnValue;
    public Light(Hardware hardware, Telemetry telemetry) {
        super(hardware, telemetry);
        state = State.PASSIVE;
        lightValue = 0;
        flashTimer = new ElapsedTime();

        setPassiveValue(params.off);
        setFlashValues(params.off, params.dim);
    }
    @Override
    public void declareHardware() {
        light = hardware.getLight();
    }

    @Override
    public void updateState() {
        switch (state) {
            case PASSIVE:
                if(robot.shooter.getAvgMotorSpeed() < robot.shooter.getMinAvgMotorSpeed()) {
                    state = State.FLASH;
                    flashTimer.reset();
                    break;
                }
                light.setPosition(passiveValue);
                break;
            case FLASH:
                if (flashTimer.seconds() > params.flashTime * 2)
                    flashTimer.reset();
                if (flashTimer.seconds() > params.flashTime)
                    lightValue = flashOffValue;
                else
                    lightValue = flashOnValue;

                light.setPosition(lightValue);
                break;
        }
    }

    public void setState(State state) {
        this.state = state;
    }
    public void setPassiveValue(double value) {
        passiveValue = value;
    }
    public void setFlashValues(double off, double on) {
        flashOffValue = off;
        flashOnValue = on;
    }

    @Override
    public void printInfo() {

    }
}