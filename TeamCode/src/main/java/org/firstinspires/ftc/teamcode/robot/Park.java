package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opModesCompetition.tele.Keybinds;
import org.firstinspires.ftc.teamcode.utils.misc.MathUtils;
import org.firstinspires.ftc.teamcode.utils.stateManagement.Subsystem;

public class Park extends Subsystem {
    public static double stowPosition = 0.01, parkPosition = 0.99;
    private ServoImplEx leftServo, rightServo;
    private boolean enteredParkMode, up;
    public Park(Hardware hardware, Telemetry telemetry) {
        super(hardware, telemetry);
        enteredParkMode = false;
        this.up = false;
    }

    @Override
    public void declareHardware() {
        leftServo = hardware.getLeftParkServo();
        rightServo = hardware.getRightParkServo();
        setServoPositions(stowPosition);
    }

    @Override
    public void updateState() {
        if (keybinds.check(Keybinds.D2Trigger.RAISE_PARK) && enteredParkMode) {
            up = !up;
            setServoPositions(up ? parkPosition : stowPosition);
        }

        if (!enteredParkMode && keybinds.check(Keybinds.D2Trigger.ENTER_PARK_MODE)) {
            robot.drivetrain.setState(Drivetrain.State.TELE_SLOW_DRIVE);
            robot.drivetrain.setParkSlowDriveScale();
            robot.shooter.setState(Shooter.State.OFF);
            enteredParkMode = true;
        }
    }

    @Override
    public void printInfo() {
        telemetry.addLine("===RAISE_PARK===");
        telemetry.addData("left servo pos", MathUtils.format3(leftServo.getPosition()));
        telemetry.addData("right servo pos", MathUtils.format3(rightServo.getPosition()));
    }
    public void setServoPositions(double position) {
        leftServo.setPosition(position);
        rightServo.setPosition(position);
    }
}
