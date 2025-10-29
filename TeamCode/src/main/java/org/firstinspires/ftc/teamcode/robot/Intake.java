package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.generalOpModes.Keybinds;
import org.firstinspires.ftc.teamcode.utils.misc.LineEquation;
import org.firstinspires.ftc.teamcode.utils.stateManagement.Subsystem;

@Config
public class Intake extends Subsystem {
    public static double collectPower = 0.99, feedShooterTime = 0.1, feedShooterPower = 0.5;

    public enum State {
        ON, OFF, FEED_SHOOTER
    }

    private State state;
    private DcMotorEx motor;
    private int numBalls;
    private final ElapsedTime feedShooterTimer;

    public Intake(Hardware hardware, Telemetry telemetry) {
        super(hardware, telemetry);
        numBalls = 0;
        state = State.OFF;
        feedShooterTimer = new ElapsedTime();
    }

    @Override
    public void declareHardware() {
        motor = hardware.getIntakeMotor();
    }

    @Override
    public void updateState() {
        switch (state) {
            case OFF:
                if (keybinds.check(Keybinds.D1Trigger.TURN_ON_INTAKE) && numBalls < 3) {
                    setState(State.ON);
                    break;
                }
                break;
            case ON:
                if (!keybinds.check(Keybinds.D1Trigger.TURN_ON_INTAKE)) {
                    setState(State.OFF);
                    break;
                }

                // update number of balls in transfer
                if (robot != null && robot.colorSensors[numBalls].firstTimeSeeingBallFromLatestCache()) {
                    numBalls++;
                    if (numBalls < 3)
                        turnOnSensor(numBalls);
                    else {
                        setState(State.OFF);
                        robot.shooter.setState(Shooter.State.TRACK_SHOOTER_SPEED); // start speeding up the shooter
                        break;
                    }
                }
                break;
            case FEED_SHOOTER:
                if (feedShooterTimer.seconds() > feedShooterTime)
                    setState(State.OFF);
        }
    }
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
            turnOffAllSensors();
        }
        else if (state == State.FEED_SHOOTER) {
            feedShooterTimer.reset();
            motor.setPower(feedShooterPower);
            turnOnSensor(numBalls);
        }
    }

    // turn on the appropriate sensor
    private void turnOnSensor(int numBalls) {
        if (robot != null)
            for(int i = 0; i<3;i++)
                robot.colorSensors[i].setTurnedOn(i == numBalls);
    }
    private void turnOffAllSensors() {
        if (robot != null)
            for(int i = 0; i<3;i++)
                robot.colorSensors[i].setTurnedOn(false);
    }

    public int getNumBalls() {
        return numBalls;
    }
    public void printInfo() {
        telemetry.addLine("===INTAKE===");
        telemetry.addData("state", state);
        telemetry.addData("keybind activated", keybinds.check(Keybinds.D1Trigger.TURN_ON_INTAKE));
        telemetry.addData("motor power", motor.getPower());
        telemetry.addData("num balls", numBalls);
    }
}
