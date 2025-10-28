package org.firstinspires.ftc.teamcode.robot;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.commands.InstantCommand;
import org.firstinspires.ftc.teamcode.utils.commands.WaitUntilCommand;
import org.firstinspires.ftc.teamcode.utils.generalOpModes.Keybinds;
import org.firstinspires.ftc.teamcode.utils.generalOpModes.GamepadTracker;
import org.firstinspires.ftc.teamcode.utils.pinpoint.Pinpoint;
import org.firstinspires.ftc.teamcode.utils.stateManagement.Sensor;
import org.firstinspires.ftc.teamcode.utils.stateManagement.Subsystem;

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
    public final Pinpoint odo;
    public final Intake intake;
    public final Flipper flipper;
    public final Shooter shooter;
    public final Park park;
    public Robot(Hardware hardware, Telemetry telemetry) {
        this.telemetry = telemetry;
        subsystems = new ArrayList<>();
        sensors = new ArrayList<>();

        odo = new Pinpoint(hardware.hardwareMap);
        camera = new Camera(hardware, telemetry);
        subsystems.add(camera);
        drivetrain = new Drivetrain(hardware, telemetry);
        subsystems.add(drivetrain);
        intake = new Intake(hardware, telemetry);
        subsystems.add(intake);
        flipper = new Flipper(hardware, telemetry);
        subsystems.add(flipper);
        shooter = new Shooter(hardware, telemetry);
        subsystems.add(shooter);
        park = new Park(hardware, telemetry);
        subsystems.add(park);

        colorSensors = new BallColorSensor[3];
        BallColorSensor backColorSensor = new BallColorSensor(hardware, telemetry, Hardware.backColorSensorName, false);
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

        for (Subsystem subsystem : subsystems) {
            subsystem.setInputInfo(keybinds);
            subsystem.setRobot(this);
        }
    }

    public void declareHardware() {
        for (Subsystem subsystem : subsystems)
            subsystem.declareHardware();
        for (Sensor sensor : sensors)
            sensor.declareHardware();
    }

    public void update() {
        // update ALL sensors before any subsystems
        odo.update();
        for (Sensor sensor : sensors)
            sensor.update();

        // update subsystems
        for (Subsystem subsystem : subsystems)
            subsystem.updateState();
    }

    public SequentialCommandGroup shootBallCommand() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> intake.setState(Intake.State.OFF)),
                new WaitUntilCommand(shooter::isReadyToShoot, Shooter.maxMotorSpeedUpTime),
                new InstantCommand(() -> flipper.setState(Flipper.State.OPEN)),
                new WaitCommand(Flipper.rotationTimeMs),
                new InstantCommand(() -> intake.setState(Intake.State.FEED_SHOOTER)),
                new InstantCommand(() -> flipper.setState(Flipper.State.CLOSED)),
                new WaitCommand(Flipper.rotationTimeMs)
        );
    }
}