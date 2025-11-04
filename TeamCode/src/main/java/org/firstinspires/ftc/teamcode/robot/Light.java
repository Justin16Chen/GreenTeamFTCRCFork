package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PwmControl;
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
        public double minPwm = 500, maxPwm = 2520;
        public double off = inverseLerp(minPwm, maxPwm, 600);
        public double bright = inverseLerp(minPwm, maxPwm, 1000);
        public double dim = inverseLerp(minPwm, maxPwm, 750);
        public double flashTime = 0.8;
    }
    public static Params params = new Params();
    private ServoImplEx light;
    private double lightValue;
    public enum LightState {
        SET,
        FLASH
    }
    private LightState lightState;
    private ElapsedTime flashTimer;
    public Light(Hardware hardware, Telemetry telemetry) {
        super(hardware, telemetry);
        lightState = LightState.SET;
        lightValue = 0;
        flashTimer = new ElapsedTime();
    }
    @Override
    public void declareHardware() {
        light = hardware.hardwareMap.get(ServoImplEx.class, "light");
        light.setPwmRange(new PwmControl.PwmRange(params.minPwm, params.maxPwm));
    }

    @Override
    public void updateState() {
        if(robot.shooter.getAvgMotorSpeed() < robot.shooter.getMinAvgMotorSpeed() && lightState != LightState.FLASH) {
            lightState = LightState.FLASH;
            flashTimer.reset();
        }
//        else if(robot.indexer.getLightTimerSeconds() < Indexer.params.lightFlashTime) {
//            lightState = LightState.SET;
//            lightValue = params.dim;
//        }
        else {
            lightValue = params.off;
            lightState = LightState.SET;
        }
        updateLight();
    }

    @Override
    public void printInfo() {

    }
    private void updateLight() {
        if (lightState == LightState.FLASH) {
            if (flashTimer.seconds() > params.flashTime)
                lightValue = params.off;
            else
                lightValue = params.bright;
            if (flashTimer.seconds() > params.flashTime * 2)
                flashTimer.reset();
        }
        light.setPosition(lightValue);
    }
    private double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }
    public double getLightValue() {
        return lightValue;
    }
    public void setLightState(LightState lightState) {
        this.lightState = lightState;
    }
    public void setLightValue(double value) {
        lightValue = value;
    }
}