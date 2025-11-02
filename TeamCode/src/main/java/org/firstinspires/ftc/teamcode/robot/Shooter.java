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
        public double shooterKp = 0.1, shooterKi = 0, shooterKd = 0, shooterKf = 0.03;
        public double velocityErrorThreshold = Math.toRadians(10), hoodErrorThreshold = 0.01;
        public double nearZoneTargetDPS = 200;
    }
    public static ShootingTuning shooterParams = new ShootingTuning();
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
    private DcMotorEx leftMotor, rightMotor;
    private ServoImplEx leftServo, rightServo;
    private final PIDFController speedPidf; // input error between current speed and target speed, output desired power
    private double targetHoodPos, pidMotorPower;
    public Shooter(Hardware hardware, Telemetry telemetry) {
        super(hardware, telemetry);
        speedPidf = new PIDFController(shooterParams.shooterKp, shooterParams.shooterKi, shooterParams.shooterKd, shooterParams.shooterKf);
        speedPidf.setOutputBounds(0, 0.99);
        setState(State.TRACK_PASSIVE_SPEED);
    }

    @Override
    public void declareHardware() {
        leftMotor = hardware.getLeftShooterMotor();
        rightMotor = hardware.getRightShooterMotor();
        leftServo = hardware.getLeftHoodServo();
        rightServo = hardware.getRightHoodServo();
    }

    @Override
    public void updateState() {
        switch (state) {
            case OFF:
                break;
            case TRACK_PASSIVE_SPEED:
                if (keybinds.check(Keybinds.D1Trigger.PREPARE_FLYWHEEL)) {
                    setState(State.TRACK_SHOOTER_SPEED);
                    break;
                }

                setMotorPowers(passivePower);
                break;
            case TRACK_SHOOTER_SPEED:
                if (keybinds.check(Keybinds.D1Trigger.SHOOT) && robot != null)
                    robot.shootBallCommand().schedule();

                pidMotorPower = speedPidf.update(getAvgMotorSpeed());
                setMotorPowers(pidMotorPower);

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
            leftMotor.setPower(0);
            setServoPositions(hoodDownPosition);
        }
        else if (state == State.TRACK_PASSIVE_SPEED) {
            speedPidf.reset();
            speedPidf.setTarget(passiveDPS);
        }
        else if (state == State.TRACK_SHOOTER_SPEED) {
            speedPidf.reset();
            speedPidf.setTarget(shooterParams.nearZoneTargetDPS);
        }
    }

//    public boolean isReadyToShoot() {
//        double currentVelocity = leftMotor.getVelocity(AngleUnit.RADIANS);
//        double velocityError = Math.abs(currentVelocity - targetSpeed);
//        double hoodError = Math.abs(getAvgServoPosition() - targetHoodPos);
//        return velocityError < shooterParams.velocityErrorThreshold && hoodError < shooterParams.hoodErrorThreshold;
//    }
    private void setMotorPowers(double power) {
        leftMotor.setPower(power);
        rightMotor.setPower(power);
    }
    public double getAvgMotorSpeed() {
        double leftSpeed = Math.abs(leftMotor.getVelocity(AngleUnit.DEGREES));
        double rightSpeed = Math.abs(rightMotor.getVelocity(AngleUnit.DEGREES));
        return leftSpeed + rightSpeed / 2;
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
        telemetry.addLine();
        telemetry.addData("pid motor power", pidMotorPower);
        telemetry.addData("target speed", shooterParams.nearZoneTargetDPS);
        telemetry.addData("actual avg speed", MathUtils.format3(getAvgMotorSpeed()));
        telemetry.addData("left speed", leftMotor.getVelocity(AngleUnit.DEGREES));
        telemetry.addData("right speed", rightMotor.getVelocity(AngleUnit.DEGREES));
        telemetry.addLine();
        telemetry.addData("left motor power", MathUtils.format3(leftMotor.getPower()));
        telemetry.addData("right motor power", MathUtils.format3(rightMotor.getPower()));
        telemetry.addData("left servo position", MathUtils.format3(leftServo.getPosition()));
        telemetry.addData("right servo position", MathUtils.format3(rightServo.getPosition()));
    }
}
