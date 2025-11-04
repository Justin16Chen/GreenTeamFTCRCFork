package org.firstinspires.ftc.teamcode.utils.stateManagement;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Hardware;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.utils.generalOpModes.GamepadTracker;
import org.firstinspires.ftc.teamcode.opModesCompetition.tele.Keybinds;

// all subsystems should inherit this class (or indirectly inherit it by extending StateSubsystem)
// stores basic necessities like gamepad trackers and the robot
public abstract class Subsystem {
    protected final Hardware hardware;
    protected final Telemetry telemetry;
    protected Robot robot;
    protected GamepadTracker g1, g2;
    protected Keybinds keybinds;
    public Subsystem(Hardware hardware, Telemetry telemetry) {
        this.hardware = hardware;
        this.telemetry = telemetry;
    }

    public void setRobot(Robot robot) {
        this.robot = robot;
    }
    public void setInputInfo(Keybinds keybinds) {
        this.g1 = keybinds.g1;
        this.g2 = keybinds.g2;
        this.keybinds = keybinds;
    }

    public abstract void declareHardware();

    public abstract void updateState();

    public abstract void printInfo();
}