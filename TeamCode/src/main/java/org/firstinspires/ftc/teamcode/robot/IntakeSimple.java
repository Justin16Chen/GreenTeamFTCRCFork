package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opModesCompetition.tele.Keybinds;
import org.firstinspires.ftc.teamcode.utils.misc.MathUtils;
import org.firstinspires.ftc.teamcode.utils.stateManagement.Subsystem;

@Config
public class IntakeSimple extends Subsystem {
    public static double preciseTrackingValidationFrames = 5;
    public static double collectPower = 0.99, passivePower = 0.6, feedShooterPower = 0.75;

    public enum State {
        ON, OFF, PASSIVE_INTAKE, FEED_SHOOTER_PRECISE, FEED_SHOOTER_TELE_TOGGLE
    }

    private State state;
    private DcMotorEx motor;

    private int numConsecutiveValidatedFrames;
    private int officialNumBalls, unofficalNumBalls;
    public IntakeSimple(Hardware hardware, Telemetry telemetry) {
        super(hardware, telemetry);
        state = State.OFF;
    }

    @Override
    public void declareHardware() {
        motor = hardware.getIntakeMotor();
    }

    @Override
    public void updateState() {
        switch (state) {
            case OFF:
                if (keybinds.check(Keybinds.D1Trigger.TOGGLE_INTAKE)) {
                    setState(State.ON);
                    break;
                }

                motor.setPower(0);
                break;
            case ON:
                if (keybinds.check(Keybinds.D1Trigger.TOGGLE_INTAKE)) {
                    setState(State.OFF);
                    break;
                }

                motor.setPower(collectPower);

                break;
            case PASSIVE_INTAKE:
                motor.setPower(passivePower);
                break;
            case FEED_SHOOTER_TELE_TOGGLE:
                if (!keybinds.check(Keybinds.D1Trigger.CONTINUE_SHOOTING)) {
                    setState(State.OFF);
                    break;
                }

                motor.setPower(feedShooterPower);
                break;
            case FEED_SHOOTER_PRECISE:
                updateNumBalls();
                if (officialNumBalls == 0) {
                    setState(State.OFF);
                    break;
                }

                motor.setPower(feedShooterPower);
                break;
        }
    }
    public State getState() { return state; }
    public void setState(State newState) {
        if (state == newState)
            return;

        state = newState;

        if (state == State.ON)
            motor.setPower(collectPower);
        else if (state == State.OFF)
            motor.setPower(0);
        else if (state == State.PASSIVE_INTAKE)
            motor.setPower(passivePower);
        else if (state == State.FEED_SHOOTER_PRECISE) {
            officialNumBalls = -1;
            motor.setPower(feedShooterPower);
        }
    }

    public void printInfo() {
        telemetry.addLine("===INTAKE===");
        telemetry.addData("state", state);
        telemetry.addData("keybind activated", keybinds.check(Keybinds.D1Trigger.TOGGLE_INTAKE));
        telemetry.addData("motor power", MathUtils.format3(motor.getPower()));
    }
    private int getCurFrameNumBalls() {
        int curFrameNumBalls = 0;
        for (BallColorSensor sensor : robot.colorSensors)
            if (sensor.seesBallFromLatestCache())
                curFrameNumBalls++;
        return curFrameNumBalls;
    }

    private void updateNumBalls() {
        // track number of frames that a consistent reading has happened
        int newNumBalls = getCurFrameNumBalls();
        if (newNumBalls == unofficalNumBalls)
            numConsecutiveValidatedFrames++;
        else
            numConsecutiveValidatedFrames = 0;

        // update latest unofficial value
        unofficalNumBalls = newNumBalls;

        // update official value
        if (numConsecutiveValidatedFrames >= preciseTrackingValidationFrames)
            officialNumBalls = unofficalNumBalls;
    }
}
