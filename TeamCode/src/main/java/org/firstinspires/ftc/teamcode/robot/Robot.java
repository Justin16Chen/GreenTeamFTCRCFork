package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.commands.InstantCommand;
import org.firstinspires.ftc.teamcode.utils.generalOpModes.Alliance;
import org.firstinspires.ftc.teamcode.utils.generalOpModes.Keybinds;
import org.firstinspires.ftc.teamcode.utils.generalOpModes.GamepadTracker;
import org.firstinspires.ftc.teamcode.utils.pinpoint.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.utils.stateManagement.Sensor;
import org.firstinspires.ftc.teamcode.utils.stateManagement.Subsystem;

import java.util.ArrayList;

@Config
public class Robot {
    public static Alliance defaultAlliance = Alliance.BLUE;

    public final Alliance alliance;
    public final ArrayList<Subsystem> subsystems;
    public final ArrayList<Sensor> sensors;
    public final BallColorSensor[] colorSensors;
    public final Drivetrain drivetrain;
    public final PinpointLocalizer pinpoint;
    public final Intake intake;
    public final Flipper flipper;
    public final Shooter shooter;
    public final Park park;
    public Robot(Hardware hardware, Telemetry telemetry) {
        this(hardware, telemetry, defaultAlliance);
    }
    public Robot(Hardware hardware, Telemetry telemetry, Alliance alliance) {
        this.alliance = alliance;
        subsystems = new ArrayList<>();
        sensors = new ArrayList<>();

        pinpoint = new PinpointLocalizer(hardware.hardwareMap, new Pose2d(0, 0, 0), telemetry);
        drivetrain = new Drivetrain(hardware, telemetry);
        subsystems.add(drivetrain);
        intake = new Intake(hardware, telemetry);
        subsystems.add(intake);
        shooter = new Shooter(hardware, telemetry);
        subsystems.add(shooter);
        flipper = new Flipper(hardware, telemetry);
        subsystems.add(flipper);
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

    public void setInputInfo(Keybinds keybinds) {
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
        pinpoint.update();
        for (Sensor sensor : sensors)
            sensor.update();

        // update subsystems
        for (Subsystem subsystem : subsystems)
            subsystem.updateState();
    }

    public SequentialCommandGroup shootBallCommand() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> intake.setState(Intake.State.OFF)),
//                new WaitUntilCommand(shooter::isReadyToShoot, Shooter.shooterParams.maxMotorSpeedUpTime),
                new InstantCommand(() -> flipper.setState(Flipper.State.OPEN)),
                new WaitCommand(Flipper.rotationTimeMs),
                new InstantCommand(() -> intake.setState(Intake.State.FEED_SHOOTER)),
                new InstantCommand(() -> flipper.setState(Flipper.State.CLOSED)),
                new WaitCommand(Flipper.rotationTimeMs),
                new InstantCommand(() -> shooter.setState(Shooter.State.TRACK_PASSIVE_SPEED))
        );
    }
}