package org.firstinspires.ftc.teamcode.utils;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Hardware;
import org.firstinspires.ftc.teamcode.robot.Robot;

public abstract class Sensor {
    protected final Hardware hardware;
    protected final Telemetry telemetry;
    private Robot robot;
    private boolean turnedOn; // whether the sensor is allowed to track data or not (for efficiency one might want to set turnedOn to false)

    public Sensor(Hardware hardware, Telemetry telemetry, boolean turnedOn) {
        this.hardware = hardware;
        this.telemetry = telemetry;
        this.turnedOn = turnedOn;
    }

    public void setRobot(Robot robot) {
        this.robot = robot;
    }

    public abstract void declareHardware();

    protected abstract void cacheReading();

    public Robot getRobot() {
        return robot;
    }
    public boolean isTurnedOn() {
        return turnedOn;
    }

    public void update() {
        if (turnedOn)
            cacheReading();
    }

    public void setTurnedOn(boolean turnOn) {
        this.turnedOn = turnOn;
    }
}
