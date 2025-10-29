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

@Config
public class Shooter extends Subsystem {

    public static class ShootingTuning {
        public double maxMotorSpeedUpTime = 5;
        public double shooterKp = 0.01, shooterKi = 0, shooterKd = 0, shooterKf = 0;
        public double velocityErrorThreshold = Math.toRadians(10), hoodErrorThreshold = 0.01;
    }
    public static ShootingTuning st = new ShootingTuning();
    public static double passiveDPS = 60, passivePower = 0.3;
    public static double manualHoodIncrementPercent = 0.02;
    public static double hoodDownPosition = 0.99, hoodUpPosition = 0.4;
    public static QuadraticEquation hoodEquation = new QuadraticEquation(1, 1, 1);
    public enum State {
        OFF,
        TRACK_PASSIVE_SPEED,
        TRACK_SHOOTER_SPEED
    }
    private State state;
    private DcMotorEx motor;
    private ServoImplEx leftServo, rightServo;
    private final PIDFController speedPidf; // input error between current speed and target speed, output desired power
    private double targetVelocity, targetHoodPos;
    public Shooter(Hardware hardware, Telemetry telemetry) {
        super(hardware, telemetry);
        speedPidf = new PIDFController(st.shooterKp, st.shooterKi, st.shooterKd, st.shooterKf);
        targetVelocity = 0;

        setState(State.TRACK_PASSIVE_SPEED);
    }

    @Override
    public void declareHardware() {
        motor = hardware.getShooterMotor();
        leftServo = hardware.getLeftHoodServo();
        rightServo = hardware.getRightHoodServo();
    }

    @Override
    public void updateState() {
        double currentVelocity = motor.getVelocity(AngleUnit.RADIANS);
        switch (state) {
            case OFF:
                break;
            case TRACK_PASSIVE_SPEED:
                if (keybinds.check(Keybinds.D1Trigger.PREPARE_FLYWHEEL)) {
                    setState(State.TRACK_SHOOTER_SPEED);
                    break;
                }

//                double passivePower = speedPidf.update(currentVelocity);
//                motor.setPower(passivePower);
                motor.setPower(passivePower);
                break;
            case TRACK_SHOOTER_SPEED:
                if (keybinds.check(Keybinds.D1Trigger.SHOOT) && robot != null)
                    robot.shootBallCommand().schedule();

//                double shootingPower = speedPidf.update(currentVelocity);
//                motor.setPower(shootingPower);
//                targetHoodPos = hoodEquation.calculate(currentVelocity);
//                setServoPositions(targetHoodPos);
                motor.setPower(0.99);
                if (keybinds.g1.isDpadUpPressed())
                    targetHoodPos += manualHoodIncrementPercent;
                else if (keybinds.g1.isDpadDownPressed())
                    targetHoodPos -= manualHoodIncrementPercent;
                setServoPositions(targetHoodPos);
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
        else if (state == State.TRACK_PASSIVE_SPEED) {
            speedPidf.reset();
            speedPidf.setTarget(passiveDPS);
        }
        else if (state == State.TRACK_SHOOTER_SPEED) {
            speedPidf.reset();
            speedPidf.setTarget(targetVelocity);
        }
    }

    public boolean isReadyToShoot() {
        double currentVelocity = motor.getVelocity(AngleUnit.RADIANS);
        double velocityError = Math.abs(currentVelocity - targetVelocity);
        double hoodError = Math.abs(getAvgServoPosition() - targetHoodPos);
        return velocityError < st.velocityErrorThreshold && hoodError < st.hoodErrorThreshold;
    }
    public double getAvgServoPosition() {
        return (leftServo.getPosition() + rightServo.getPosition()) / 2.;
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
