package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.misc.MathUtils;
import org.firstinspires.ftc.teamcode.utils.stateManagement.Subsystem;

@Config
public class Flipper extends Subsystem {
    public static double closePosition = 0.01, openPosition = 0.95;
    public static long rotationTimeMs = 150;
    public enum State {
        OPEN, CLOSED
    }
    private State state;
    private ServoImplEx servo;
    private final ElapsedTime stateTimer;
    public Flipper(Hardware hardware, Telemetry telemetry) {
        super(hardware, telemetry);
        state = State.CLOSED;
        stateTimer = new ElapsedTime();
        stateTimer.reset();
    }

    @Override
    public void declareHardware() {
        servo = hardware.getFlipperServo();
        servo.setPosition(closePosition);
    }

    @Override
    public void updateState() {}

    public State getState() {
        return state;
    }
    public boolean isMoving() {
        return stateTimer.milliseconds() < rotationTimeMs;
    }

    public void setState(State newState) {
        if (state == newState)
            return;

        state = newState;
        stateTimer.reset();

        if (state == State.OPEN)
            servo.setPosition(openPosition);
        else if (state == State.CLOSED)
            servo.setPosition(closePosition);
    }

    @Override
    public void printInfo() {
        telemetry.addLine("===FLIPPER===");
        telemetry.addData("state", state);
        telemetry.addData("target servo position", MathUtils.format3(servo.getPosition()));
    }
}
