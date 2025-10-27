package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.stateManagement.StateSubsystem;

@Config
public class Flipper extends StateSubsystem<Flipper.State> {
    public static double blockPosition = 0.01, openPosition = 0.99;
    public static long rotationTimeMs = 200;
    public enum State {
        OPEN, CLOSED
    }
    private ServoImplEx servo;
    public Flipper(Hardware hardware, Telemetry telemetry) {
        super(hardware, telemetry);
        setInitialState(State.OPEN);

        setTransitionFunction(State.CLOSED, State.OPEN, () -> servo.setPosition(openPosition));
        setTransitionFunction(State.OPEN, State.CLOSED, () -> servo.setPosition(blockPosition));
    }

    @Override
    public void declareHardware() {
        servo = hardware.getFlipperServo();
    }

    @Override
    public void updateState() {
        if (g1.isLBClicked())
            setState(getState() == State.OPEN ? State.CLOSED : State.OPEN);
    }

    @Override
    public void printInfo() {

    }
}
