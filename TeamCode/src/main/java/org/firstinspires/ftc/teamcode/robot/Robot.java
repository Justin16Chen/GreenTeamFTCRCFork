package org.firstinspires.ftc.teamcode.robot;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opModesCompetition.tele.Keybinds;
import org.firstinspires.ftc.teamcode.utils.BallColorSensor;
import org.firstinspires.ftc.teamcode.utils.GamepadTracker;
import org.firstinspires.ftc.teamcode.utils.Sensor;
import org.firstinspires.ftc.teamcode.utils.Subsystem;

import java.util.ArrayList;

public class Robot {

    private final Telemetry telemetry;
    private GamepadTracker g1, g2;
    private Keybinds keybinds;
    public final ArrayList<Subsystem> subsystems;
    public final ArrayList<Sensor> sensors;
    public final Camera camera;
    public final BallColorSensor[] colorSensors;
    public final Drivetrain drivetrain;
    public final Intake intake;
    public final Park park;
    public Robot(Hardware hardware, Telemetry telemetry) {
        this.telemetry = telemetry;
        subsystems = new ArrayList<>();
        sensors = new ArrayList<>();

        camera = new Camera(hardware, telemetry);
        subsystems.add(camera);
        drivetrain = new Drivetrain(hardware, telemetry);
        subsystems.add(drivetrain);
        intake = new Intake(hardware, telemetry);
        subsystems.add(intake);
        park = new Park(hardware, telemetry);
        subsystems.add(park);

        colorSensors = new BallColorSensor[3];
        BallColorSensor backColorSensor = new BallColorSensor(hardware, telemetry, Hardware.backColorSensorName, true);
        sensors.add(backColorSensor);
        colorSensors[0] = backColorSensor;
        BallColorSensor middleColorSensor = new BallColorSensor(hardware, telemetry, Hardware.middleColorSensorName, false);
        sensors.add(middleColorSensor);
        colorSensors[1] = middleColorSensor;
        BallColorSensor frontColorSensor = new BallColorSensor(hardware, telemetry, Hardware.frontColorSensorName, false);
        sensors.add(frontColorSensor);
        colorSensors[2] = frontColorSensor;

        for (Subsystem subsystem : subsystems)
            subsystem.setRobot(this);

        for (Sensor sensor : sensors)
            sensor.setRobot(this);
    }

    public void setInputInfo(GamepadTracker g1, GamepadTracker g2, Keybinds keybinds) {
        this.g1 = g1;
        this.g2 = g2;

        for (Subsystem subsystem : subsystems)
            subsystem.setInputInfo(g1, g2, keybinds);
    }

    public void declareHardware() {
        for (Subsystem subsystem : subsystems)
            subsystem.declareHardware();
        for (Sensor sensor : sensors)
            sensor.declareHardware();
    }

    public void update() {
        // update ALL sensors before any subsystems
        for (Sensor sensor : sensors)
            sensor.update();

        // update subsystems
        for (Subsystem subsystem : subsystems)
            subsystem.updateState();
    }
}