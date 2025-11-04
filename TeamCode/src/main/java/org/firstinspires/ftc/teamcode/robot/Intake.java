package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opModesCompetition.tele.Keybinds;
import org.firstinspires.ftc.teamcode.utils.misc.MathUtils;
import org.firstinspires.ftc.teamcode.utils.stateManagement.Subsystem;

@Config
public class Intake extends Subsystem {
    public static int noMoreBallsValidationFrames = 5;
    public static double collectPower = 0.99, passivePower = 0.6, threeBallPassivePower = 0.4, feedShooterPower = 0.75;

    public enum State {
        ON, OFF, PASSIVE_INTAKE, FEED_SHOOTER
    }

    private State state;
    private DcMotorEx motor;
    private int numBalls;
    private int numConsecutiveValidatedFrames;
    private int unofficalNumBalls;

    public Intake(Hardware hardware, Telemetry telemetry) {
        super(hardware, telemetry);
        numBalls = 0;
        state = State.OFF;
        numConsecutiveValidatedFrames = 0;
    }

    @Override
    public void declareHardware() {
        motor = hardware.getIntakeMotor();
    }

    @Override
    public void updateState() {
        switch (state) {
            case OFF:
                if (keybinds.check(Keybinds.D1Trigger.TOGGLE_INTAKE) && numBalls < 3) {
                    setState(State.ON);
                    break;
                }
                break;
            case ON:
                if (keybinds.check(Keybinds.D1Trigger.TOGGLE_INTAKE)) {
                    setState(numBalls == 0 ? State.OFF : State.PASSIVE_INTAKE);
                    break;
                }

                // update number of balls in transfer
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
                // track number of frames that a consistent reading has happened
                int newNumBalls = getCurFrameNumBalls();
                if (newNumBalls == unofficalNumBalls)
                    numConsecutiveValidatedFrames++;
                else
                    numConsecutiveValidatedFrames = 0;

                // update latest unofficial value
                unofficalNumBalls = newNumBalls;

                // update official value
                if (numConsecutiveValidatedFrames >= noMoreBallsValidationFrames) {
                    numBalls = unofficalNumBalls;

                    if (numBalls == 0) {
                        setState(State.OFF);
                        break;
                    }
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

        if (state == State.ON) {
            motor.setPower(collectPower);
            turnOnSensor(numBalls);
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
            numConsecutiveValidatedFrames = 0;
            unofficalNumBalls = numBalls;
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
}
