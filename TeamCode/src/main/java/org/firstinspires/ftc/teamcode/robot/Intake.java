package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opModesCompetition.tele.Keybinds;
import org.firstinspires.ftc.teamcode.utils.Subsystem;

public class Intake extends Subsystem {
    public static double intakePower = 0.99;
    public enum State {
        ON, OFF
    }
    private DcMotorEx motor;
    private State state;
    public Intake(Hardware hardware, Telemetry telemetry) {
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
}
