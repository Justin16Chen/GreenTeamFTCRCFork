package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.commands.InstantCommand;
import org.firstinspires.ftc.teamcode.utils.commands.WaitUntilCommand;
import org.firstinspires.ftc.teamcode.utils.generalOpModes.Alliance;
import org.firstinspires.ftc.teamcode.opModesCompetition.tele.Keybinds;
import org.firstinspires.ftc.teamcode.utils.generalOpModes.OpmodeType;
import org.firstinspires.ftc.teamcode.utils.pinpoint.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.utils.stateManagement.Sensor;
import org.firstinspires.ftc.teamcode.utils.stateManagement.Subsystem;

import java.util.ArrayList;

@Config
public class Robot {
    // these units are in inches
    public static double width = 13.5, length = 16.7, frontToCenterLength = 10, backToCenterLength = 6.7;
    public static Alliance defaultAlliance = Alliance.BLUE;

    public final Alliance alliance;
    public final OpmodeType opmodeType;
    public final ArrayList<Subsystem> subsystems;
    public final ArrayList<Sensor> sensors;
    public final BallColorSensor[] colorSensors;
    public final Drivetrain drivetrain;
    public final PinpointLocalizer pinpoint;
    public final Intake intake;
    public final Flipper flipper;
    public final Shooter shooter;
    public final Park park;
    public Robot(Hardware hardware, Telemetry telemetry, OpmodeType opmodeType) {
        this(hardware, telemetry, opmodeType, defaultAlliance);
    }
    public Robot(Hardware hardware, Telemetry telemetry, OpmodeType opmodeType, Alliance alliance) {
        this.opmodeType = opmodeType;
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

    public Command shootBallCommand() {
        return new ParallelCommandGroup(
                new SequentialCommandGroup(
                        new InstantCommand(() -> flipper.setState(Flipper.State.OPEN)),
                        new WaitCommand(Flipper.rotationTimeMs),
                        new InstantCommand(() -> intake.setState(Intake.State.FEED_SHOOTER)),
                        new WaitUntilCommand(() -> intake.getState() != Intake.State.FEED_SHOOTER, Shooter.maxShootTimeMs),
                        new InstantCommand(() -> flipper.setState(Flipper.State.CLOSED)),
                        new WaitCommand(Flipper.rotationTimeMs + Shooter.shooterParams.ballShootTime),
                        new InstantCommand(() -> shooter.setState(Shooter.State.TRACK_PASSIVE_SPEED))
                )
//                drivetrain.driveForwardsIfNecessary()
        );
    }
}