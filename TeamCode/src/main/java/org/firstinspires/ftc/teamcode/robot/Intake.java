package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.generalOpModes.Keybinds;
import org.firstinspires.ftc.teamcode.utils.misc.LineEquation;
import org.firstinspires.ftc.teamcode.utils.stateManagement.StateSubsystem;
import org.firstinspires.ftc.teamcode.utils.stateManagement.Transition;

@Config
public class Intake extends StateSubsystem<Intake.State> {
    public static double collectPower = 0.99, feedShooterTime = 0.1, feedShooterPowerYInt = 0.2, feedShooterPowerSlope = 0.25;
    public static LineEquation feedShooterEquation = new LineEquation(feedShooterPowerSlope, feedShooterPowerYInt);

    public enum State {
        ON, OFF, FEED_SHOOTER
    }

    private DcMotorEx motor;
    private int numBalls;

    public Intake(Hardware hardware, Telemetry telemetry) {
        super(hardware, telemetry);
        setInitialState(State.OFF);
        numBalls = 0;
        setTransitionFunction(State.OFF, State.ON, () -> {
            motor.setPower(collectPower);
            turnOnSensor(numBalls);
        });

        setTransitionFunction(State.ON, State.OFF, () -> {
            motor.setPower(0);
            turnOffAllSensors();
        });
        setTransitionFunction(Transition.Type.FROM_ANY_TO, State.FEED_SHOOTER, () -> {
            motor.setPower(feedShooterEquation.calculate(numBalls));
            turnOffAllSensors();
        });
        setTransitionFunction(Transition.Type.TO_ANY_FROM, State.FEED_SHOOTER, () -> {
            if (numBalls > 0)
                numBalls--;
        });
    }

    @Override
    public void declareHardware() {
        motor = hardware.getIntakeMotor();
    }

    @Override
    public void updateState() {
        switch (getState()) {
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
                if (robot.colorSensors[numBalls].firstTimeSeeingBallFromLatestCache()) {
                    numBalls++;

                    if (numBalls < 3)
                        turnOnSensor(numBalls);

                    else {
                        setState(State.OFF);
                        break;
                    }
                }
                break;
            case FEED_SHOOTER:
                if (getStateTime() > feedShooterTime)
                    setState(State.OFF);
        }
    }

    // turn on the appropriate sensor
    private void turnOnSensor(int numBalls) {
        for(int i = 0; i<3;i++)
            robot.colorSensors[i].setTurnedOn(i == numBalls);
    }
    private void turnOffAllSensors() {
        for(int i = 0; i<3;i++)
            robot.colorSensors[i].setTurnedOn(false);
    }

    public int getNumBalls() {
        return numBalls;
    }
    public void printInfo() {
        telemetry.addLine("===INTAKE===");
        telemetry.addData("state", getState());
        telemetry.addData("keybind activated", keybinds.check(Keybinds.D1Trigger.TURN_ON_INTAKE));
        telemetry.addData("motor power", motor.getPower());
        telemetry.addData("num balls", numBalls);
    }
}
