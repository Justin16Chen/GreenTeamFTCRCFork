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

    public static class MotorParams {
        public double maxMotorSpeedUpTime = 5;
        public double shooterKp = 0.01, shooterKi = 0, shooterKd = 0, shooterKf = 0;
        public double velocityErrorThreshold = Math.toRadians(10), hoodErrorThreshold = 0.01;
    }
    public static MotorParams mp = new MotorParams();
    public static double hoodDownPosition = 0.99, hoodUpPosition = 0.4;
    public static QuadraticEquation hoodEquation = new QuadraticEquation(1, 1, 1);
    public enum State {
        OFF,
        TRACK_SPEED
    }
    private State state;
    private DcMotorEx motor;
    private ServoImplEx leftServo, rightServo;
    private final PIDFController speedPidf; // input error between current speed and target speed, output desired power
    private double targetVelocity, targetHoodPos;
    public Shooter(Hardware hardware, Telemetry telemetry) {
        super(hardware, telemetry);
        speedPidf = new PIDFController(mp.shooterKp, mp.shooterKi, mp.shooterKd, mp.shooterKf);
        targetVelocity = 0;

        setState(State.TRACK_SPEED);
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
                double power = speedPidf.update(velocity);
                motor.setPower(power);
                targetHoodPos = hoodEquation.calculate(velocity);
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
        else if (state == State.TRACK_SPEED) {
            speedPidf.reset();
            speedPidf.setTarget(targetVelocity);
        }
    }

    public boolean isReadyToShoot() {
        double currentVelocity = motor.getVelocity(AngleUnit.RADIANS);
        double velocityError = Math.abs(currentVelocity - targetVelocity);
        double hoodError = Math.abs(getAvgServoPosition() - targetHoodPos);
        return velocityError < mp.velocityErrorThreshold && hoodError < mp.hoodErrorThreshold;
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
