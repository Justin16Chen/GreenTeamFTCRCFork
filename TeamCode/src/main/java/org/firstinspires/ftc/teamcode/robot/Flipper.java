package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.misc.MathUtils;
import org.firstinspires.ftc.teamcode.utils.stateManagement.Subsystem;

@Config
public class Flipper extends Subsystem {
    public static double closePosition = 0.01, openPosition = 0.99;
    public static long rotationTimeMs = 200;
    public enum State {
        OPEN, CLOSED
    }
    private State state;
    private ServoImplEx servo;
    public Flipper(Hardware hardware, Telemetry telemetry) {
        super(hardware, telemetry);
        state = State.OPEN;
    }

    @Override
    public void declareHardware() {
        servo = hardware.getFlipperServo();
    }

    @Override
    public void updateState() {}

    public void setState(State newState) {
        if (state == newState)
            return;
        state = newState;
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
