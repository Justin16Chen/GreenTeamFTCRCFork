package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.utils.generalOpModes.Keybinds;
import org.firstinspires.ftc.teamcode.utils.misc.MathUtils;
import org.firstinspires.ftc.teamcode.utils.misc.PIDFController;
import org.firstinspires.ftc.teamcode.utils.misc.QuadraticEquation;
import org.firstinspires.ftc.teamcode.utils.stateManagement.Subsystem;
import org.firstinspires.ftc.teamcode.utils.stateManagement.Transition;

@Config
public class Shooter extends Subsystem {

    public static double maxMotorSpeedUpTime = 5;
    public static double shooterKp = 0.01, shooterKi = 0, shooterKd = 0, shooterKf = 0;
    public static double hoodDownPosition = 0.99, hoodUpPosition = 0.4;
    public static QuadraticEquation hoodEquation = new QuadraticEquation(1, 1, 1);
    public enum State {
        OFF,
        TRACK_SPEED
    }
    private State state;
    private DcMotorEx motor;
    private ServoImplEx leftServo, rightServo;
    private final PIDFController speedPid; // input error between current speed and target speed, output desired power
    private double targetSpeed;
    private double targetX, targetY;
    public Shooter(Hardware hardware, Telemetry telemetry) {
        super(hardware, telemetry);
        speedPid = new PIDFController(shooterKp, shooterKi, shooterKd, shooterKf);
        targetSpeed = 0;

        state = State.TRACK_SPEED;
    }

    @Override
    public void declareHardware() {
        motor = hardware.getShooterMotor();
        leftServo = hardware.getLeftHoodServo();
        rightServo = hardware.getRightHoodServo();
    }

    @Override
    public void updateState() {
        switch (state) {
            case OFF:
                break;
            case TRACK_SPEED:
                if (keybinds.check(Keybinds.D1Trigger.SHOOT) && robot != null) {
                    robot.shootBallCommand().schedule();
                }
                double velocity = motor.getVelocity(AngleUnit.RADIANS);
                double power = speedPid.update(velocity);
                motor.setPower(power);
                double hoodPos = hoodEquation.calculate(velocity);
                setServoPositions(hoodPos);
                break;
        }
    }

    public void setState(State newState) {
        if (state == newState)
            return;
        state = newState;
        if (state == State.OFF) {
            motor.setPower(0);
            setServoPositions(hoodDownPosition);
        }
    }

    public boolean isReadyToShoot() {
        return true;
    }
    private void setServoPositions(double pos) {
        double lerpedValue = MathUtils.lerp(hoodDownPosition, hoodUpPosition, pos);
        leftServo.setPosition(lerpedValue);
        rightServo.setPosition(lerpedValue);
    }

    @Override
    public void printInfo() {
        telemetry.addLine("===SHOOTER===");
        telemetry.addData("state", state);
        telemetry.addData("motor power", motor.getPower());
        telemetry.addData("left servo position", leftServo.getPosition());
        telemetry.addData("right servo position", rightServo.getPosition());
    }
}
