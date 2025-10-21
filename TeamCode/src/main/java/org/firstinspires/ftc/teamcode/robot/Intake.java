package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opModesCompetition.tele.Keybinds;
import org.firstinspires.ftc.teamcode.utils.BallColorSensor;
import org.firstinspires.ftc.teamcode.utils.Subsystem;

public class Intake extends Subsystem {
    public static double intakePower = 0.99;
    public enum State {
        ON, OFF
    }
    private DcMotorEx motor;
    private State state;
    private int numBalls;
    public Intake(Hardware hardware, Telemetry telemetry) {
        super(hardware, telemetry);
        state = State.OFF;
        numBalls = 0;
    }

    @Override
    public void declareHardware() {
        motor = hardware.getIntakeMotor();
    }

    public void updateStateSimple() {
        switch (state) {
            case OFF:
                if (keybinds.check(Keybinds.D1Trigger.TOGGLE_INTAKE)) {
                    state = State.ON;
                    motor.setPower(intakePower);
                }
                break;
            case ON:
                if (keybinds.check(Keybinds.D1Trigger.TOGGLE_INTAKE)) {
                    state = State.OFF;
                    motor.setPower(0);
                }
                break;
        }
    }
    @Override
    public void updateState() {
        switch (state) {
            case OFF:
                if (keybinds.check(Keybinds.D1Trigger.TOGGLE_INTAKE) && numBalls < 3) {
                    state = State.ON;
                    motor.setPower(intakePower);

                    // turn on the appropriate sensor
                    for (int i=0; i<3; i++)
                        robot.colorSensors[i].setTurnedOn(i == numBalls);
                    break;
                }
                break;
            case ON:
                if (keybinds.check(Keybinds.D1Trigger.TOGGLE_INTAKE)) {
                    state = State.OFF;
                    motor.setPower(0);

                    // turn off all sensors
                    for (int i=0; i<3; i++)
                        robot.colorSensors[i].setTurnedOn(false);
                    break;
                }

                // update number of balls in transfer
                if (robot.colorSensors[numBalls].firstTimeSeeingBallFromLatestCache()) {
                    numBalls++;


                    state = State.OFF;
                    motor.setPower(0);
                    for (int i=0; i<3; i++)
                        robot.colorSensors[i].setTurnedOn(false);
//                    if (numBalls < 3) {
//                        // turn on the next sensor
//                        robot.colorSensors[numBalls].setTurnedOn(true);
//                        robot.colorSensors[numBalls - 1].setTurnedOn(false);
//                    }
//                    else {
//                        // turn off all sensors
//                        state = State.OFF;
//                        motor.setPower(0);
//                        for (int i=0; i<3; i++)
//                            robot.colorSensors[i].setTurnedOn(false);
//                        break;
//                    }

                }
                break;
        }
    }

    public void printIntakeInfo() {
        telemetry.addLine("===INTAKE===");
        telemetry.addData("state", state);
        telemetry.addData("motor power", motor.getPower());
        telemetry.addData("num balls", numBalls);
    }
}
