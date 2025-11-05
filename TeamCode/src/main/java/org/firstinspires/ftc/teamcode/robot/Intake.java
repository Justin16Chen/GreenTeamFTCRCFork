package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opModesCompetition.tele.Keybinds;
import org.firstinspires.ftc.teamcode.utils.misc.MathUtils;
import org.firstinspires.ftc.teamcode.utils.stateManagement.Subsystem;

@Config
public class Intake extends Subsystem {
    public static int preciseTrackingValidationFrames = 12;
    public static double preciseTrackingThreshold = 0.7;
    public static double collectPower = 0.99, passivePower = 0.6, threeBallPassivePower = 0.4, feedShooterPower = 0.75;

    public enum State {
        ON, ON_WITH_PRECISE_BALL_TRACKING, OFF, PASSIVE_INTAKE, FEED_SHOOTER
    }

    private State state;
    private DcMotorEx motor;
    private int numBalls;
    private int numConsecutiveValidatedFrames;
    private int unofficalNumBalls;
    private int currentTrackingFrameNum;
    private int[] preciseTrackingCounts;
    private final ElapsedTime stateTimer;

    public Intake(Hardware hardware, Telemetry telemetry) {
        super(hardware, telemetry);
        numBalls = 0;
        state = State.OFF;
//        numConsecutiveValidatedFrames = 0;
        currentTrackingFrameNum = 0;
        preciseTrackingCounts = new int[3];
        stateTimer = new ElapsedTime();
        stateTimer.reset();
    }
    public void setNumBalls(int numBalls) {
        this.numBalls = numBalls;
    }

    @Override
    public void declareHardware() {
        motor = hardware.getIntakeMotor();
    }

    @Override
    public void updateState() {
        if (g1.isXClicked())
            setState(State.ON_WITH_PRECISE_BALL_TRACKING);

        switch (state) {
            case OFF:
                if (keybinds.check(Keybinds.D1Trigger.TOGGLE_INTAKE) && numBalls < 3) {
                    setState(State.ON);
                    break;
                }
                break;
            case ON_WITH_PRECISE_BALL_TRACKING:
                if (keybinds.check(Keybinds.D1Trigger.TOGGLE_INTAKE)) {
                    setState(numBalls == 0 ? State.OFF : State.PASSIVE_INTAKE);
                    break;
                }

                updateNumBallsPrecise();

                if (currentTrackingFrameNum > preciseTrackingValidationFrames) {
                    setState(State.ON);
                    break;
                }

                motor.setPower(collectPower);
                break;
            case ON:
                if (keybinds.check(Keybinds.D1Trigger.TOGGLE_INTAKE)) {
                    setState(numBalls == 0 ? State.OFF : State.PASSIVE_INTAKE);
                    break;
                }

                // update number of balls in transfer
                if (numBalls >= 3) {
                    setState(State.PASSIVE_INTAKE);
                }

                if (robot != null && robot.colorSensors[numBalls].firstTimeSeeingBallFromLatestCache()) {
                    numBalls++;
                    if (numBalls < 3)
                        turnOnSensor(numBalls);
                    else {
                        setState(State.PASSIVE_INTAKE);
                        robot.shooter.setState(Shooter.State.TRACK_SHOOTER_SPEED); // start speeding up the shooter
                        break;
                    }
                }

                motor.setPower(collectPower);

                break;
            case PASSIVE_INTAKE:
                if (keybinds.check(Keybinds.D1Trigger.TOGGLE_INTAKE)) {
                    setState(State.ON);
                    break;
                }
                motor.setPower(numBalls == 3 ? threeBallPassivePower : passivePower);
                break;
            case FEED_SHOOTER:
                updateNumBallsPrecise();

                if (numBalls == 0 && currentTrackingFrameNum >= preciseTrackingValidationFrames) {
                    setState(State.OFF);
                    break;
                }
                break;

        }
    }
    public State getState() { return state; }
    public void setState(State newState) {
        if (state == newState)
            return;

        state = newState;
        stateTimer.reset();

        if (state == State.ON) {
            motor.setPower(collectPower);
            turnOnSensor(numBalls);
        }
        else if (state == State.ON_WITH_PRECISE_BALL_TRACKING) {
            motor.setPower(collectPower);
            setAllSensorsTurnedOn(true);
            currentTrackingFrameNum = 0;
            preciseTrackingCounts = new int[3];
        }
        else if (state == State.OFF) {
            motor.setPower(0);
            setAllSensorsTurnedOn(false);
        }
        else if (state == State.PASSIVE_INTAKE) {
            motor.setPower(numBalls == 3 ? threeBallPassivePower : passivePower);
            setAllSensorsTurnedOn(false);
        }
        else if (state == State.FEED_SHOOTER) {
//            numConsecutiveValidatedFrames = 0;
//            unofficalNumBalls = numBalls;
            currentTrackingFrameNum = 0;
            preciseTrackingCounts = new int[3];
            motor.setPower(feedShooterPower);
            setAllSensorsTurnedOn(true);

        }
    }

    // turn on the appropriate sensor
    private void turnOnSensor(int numBalls) {
        if (robot != null)
            for(int i = 0; i<3;i++)
                robot.colorSensors[i].setTurnedOn(i == numBalls);
    }
    private void setAllSensorsTurnedOn(boolean turnedOn) {
        if (robot != null)
            for(int i = 0; i<3;i++)
                robot.colorSensors[i].setTurnedOn(turnedOn);
    }

    public int getNumBalls() {
        return numBalls;
    }
    public void printInfo() {
        telemetry.addLine("===INTAKE===");
        telemetry.addData("state", state);
        telemetry.addData("keybind activated", keybinds.check(Keybinds.D1Trigger.TOGGLE_INTAKE));
        telemetry.addData("motor power", MathUtils.format3(motor.getPower()));
        telemetry.addData("num balls", numBalls);
    }
    private int getCurFrameNumBalls() {
        int curFrameNumBalls = 0;
        for (BallColorSensor sensor : robot.colorSensors)
            if (sensor.seesBallFromLatestCache())
                curFrameNumBalls++;
        return curFrameNumBalls;
    }

    private void updateNumBallsOld() {
        // track number of frames that a consistent reading has happened
//        int newNumBalls = getCurFrameNumBalls();
//        if (newNumBalls == unofficalNumBalls)
//            numConsecutiveValidatedFrames++;
//        else
//            numConsecutiveValidatedFrames = 0;
//
//        // update latest unofficial value
//        unofficalNumBalls = newNumBalls;
//
//        // update official value
//        if (numConsecutiveValidatedFrames >= preciseTrackingValidationFrames)
//            numBalls = unofficalNumBalls;
    }
    private void updateNumBallsPrecise() {
        for (int i=0; i<3; i++)
            if (robot.colorSensors[i].seesBallFromLatestCache())
                preciseTrackingCounts[i]++;
        currentTrackingFrameNum++;
        if (currentTrackingFrameNum >= preciseTrackingValidationFrames) {
            numBalls = 0;
            for (int i=0; i<3; i++)
                // if the sensor saw the ball enough times, increment num balls
                if (preciseTrackingCounts[i] * 1.0 / preciseTrackingValidationFrames > preciseTrackingThreshold)
                    numBalls++;
        }
    }
}
