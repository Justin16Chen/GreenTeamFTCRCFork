package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.Keybinds;
import org.firstinspires.ftc.teamcode.utils.StateSubsystem;
import org.firstinspires.ftc.teamcode.utils.Subsystem;
import org.firstinspires.ftc.teamcode.utils.Transition;

@Config
public class Intake extends StateSubsystem<Intake.State> {
    public static double intakePower = 0.85;

    public enum State {
        ON, OFF
    }

    private DcMotorEx motor;
    private int numBalls;

    public Intake(Hardware hardware, Telemetry telemetry) {
        super(hardware, telemetry);
        setInitialState(State.OFF);
        numBalls = 0;
        setTransitionFunction(State.OFF, State.ON, () -> {
            motor.setPower(intakePower);
            turnOnSensor(numBalls);
        });

        setTransitionFunction(State.ON, State.OFF, () -> {
            motor.setPower(0);
            turnOffAllSensors();
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
                if (keybinds.check(Keybinds.D1Trigger.TOGGLE_INTAKE) && numBalls < 3) {
                    setState(State.ON);
                    break;
                }
                break;
            case ON:
                if (keybinds.check(Keybinds.D1Trigger.TOGGLE_INTAKE)) {
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

    public void printIntakeInfo() {
        telemetry.addLine("===INTAKE===");
        telemetry.addData("state", getState());
        telemetry.addData("keybind activated", keybinds.check(Keybinds.D1Trigger.TOGGLE_INTAKE));
        telemetry.addData("motor power", motor.getPower());
        telemetry.addData("num balls", numBalls);
    }
}
