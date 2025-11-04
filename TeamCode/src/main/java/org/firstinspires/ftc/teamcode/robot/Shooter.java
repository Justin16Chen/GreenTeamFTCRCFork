package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.opModesTesting.ShooterSpeedRecorder;
import org.firstinspires.ftc.teamcode.opModesCompetition.tele.Keybinds;
import org.firstinspires.ftc.teamcode.utils.misc.LineEquation;
import org.firstinspires.ftc.teamcode.utils.misc.MathUtils;
import org.firstinspires.ftc.teamcode.utils.misc.PIDFController;
import org.firstinspires.ftc.teamcode.utils.stateManagement.Subsystem;

import java.util.Collections;
import java.util.Set;

@Config
public class Shooter extends Subsystem {

    public double getMinAvgMotorSpeed() {
        return shooterParams.nearZoneMinSpeed;
    }

    public static class ShootingTuning {
        public double minPower = -0.2;
        public double shooterKp = 0.1, shooterKi = 0, shooterKd = 0, shooterKf = 0.1;
        public double speedErrorToApplyWeightingThreshold = 50;
        public double nearZoneTargetSpeed = 300, nearZoneTargetConstantWeighting = 0.7, nearZoneTargetConstant = 0.9;
        public double nearZoneMinSpeed = 280, nearZoneDriveForwardShooterSpeed = 260, nearZoneDriveForwardDrivetrainSpeed = 0.5;
        public long ballShootTime = 1000;
    }
    public static ShootingTuning shooterParams = new ShootingTuning();
    public static long maxShootTimeMs = 5000;
    public static double passivePower = 0.3, intakeOnPower = 0.75;
    public static double hoodDefaultPosition = 0.2;
    public static double hoodManualIncrementPercent = 0.02;
    public static double hoodDownPosition = 0.89, hoodUpPosition = 0.4;
    public static LineEquation hoodEquation = new LineEquation(-0.00595, 2.344);
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
    private ElapsedTime timer;
    public Shooter(Hardware hardware, Telemetry telemetry) {
        super(hardware, telemetry);
        speedPidf = new PIDFController(shooterParams.shooterKp, shooterParams.shooterKi, shooterParams.shooterKd, shooterParams.shooterKf);
        speedPidf.setOutputBounds(0, 0.99);
        setState(State.TRACK_PASSIVE_SPEED);
        timer = new ElapsedTime();
    }

    @Override
    public void declareHardware() {
        leftMotor = hardware.getLeftShooterMotor();
        rightMotor = hardware.getRightShooterMotor();
        leftServo = hardware.getLeftHoodServo();
        rightServo = hardware.getRightHoodServo();
        targetHoodPos = hoodDefaultPosition;
        setLerpedServoPositions(targetHoodPos);
    }

    @Override
    public void updateState() {
        if (keybinds.g1.isDpadUpPressed()) {
            shooterParams.nearZoneTargetSpeed += 10;
            speedPidf.setTarget(shooterParams.nearZoneTargetSpeed);
        } else if (keybinds.g1.isDpadDownPressed()) {
            shooterParams.nearZoneTargetSpeed -= 10;
            speedPidf.setTarget(shooterParams.nearZoneTargetSpeed);
        }


        switch (state) {
            case OFF:
                break;
            case TRACK_PASSIVE_SPEED:
                if (keybinds.check(Keybinds.D1Trigger.PREPARE_FLYWHEEL)) {
                    setState(State.TRACK_SHOOTER_SPEED);
                    robot.intake.setState(Intake.State.PASSIVE_INTAKE);
                    break;
                }
                setMotorPowers(passivePower);
                break;
            case TRACK_SHOOTER_SPEED:
                if (keybinds.check(Keybinds.D1Trigger.SHOOT) && robot != null) {
                    robot.shootBallCommand().schedule();
                    shooterTrackerCommand().schedule();
                }
                if (keybinds.check(Keybinds.D1Trigger.STOP_SHOOTING)) {
                    setState(State.TRACK_PASSIVE_SPEED);
                    robot.intake.setState(robot.intake.getNumBalls() == 0 ? Intake.State.OFF : Intake.State.PASSIVE_INTAKE);
                    break;
                }

                targetHoodPos = Range.clip(hoodEquation.calculate(getAvgMotorSpeed()), 0.01, 0.99);
                setRawServoPositions(targetHoodPos);

                pidMotorPower = speedPidf.update(getAvgMotorSpeed());
                // weight pid power to reduce variance once error is small enough
                if (Math.abs(shooterParams.nearZoneTargetSpeed - getAvgMotorSpeed()) < shooterParams.speedErrorToApplyWeightingThreshold)
                    pidMotorPower = MathUtils.lerp(pidMotorPower, shooterParams.nearZoneTargetConstant, shooterParams.nearZoneTargetConstantWeighting);
                pidMotorPower = Math.max(shooterParams.minPower, pidMotorPower);

                if (robot.intake.getState() == Intake.State.ON)
                    setMotorPowers(Math.min(pidMotorPower, intakeOnPower)); // give collector some lenience
                else
                    setMotorPowers(pidMotorPower);
                break;
        }
    }

    public void setState(State newState) {
        if (state == newState)
            return;
        state = newState;
        if (state == State.OFF) {
            setMotorPowers(0);
            setRawServoPositions(hoodDownPosition);
        }
        else if (state == State.TRACK_SHOOTER_SPEED) {
            speedPidf.reset();
            speedPidf.setTarget(shooterParams.nearZoneTargetSpeed);
        }
    }

//    public boolean isReadyToShoot() {
//        double currentVelocity = leftMotor.getVelocity(AngleUnit.RADIANS);
//        double velocityError = Math.abs(currentVelocity - targetSpeed);
//        double hoodError = Math.abs(getAvgServoPosition() - targetHoodPos);
//        return velocityError < shooterParams.velocityErrorThreshold && hoodError < shooterParams.hoodErrorThreshold;
//    }
    private void setMotorPowers(double power) {
        if (power < shooterParams.minPower)
            throw new IllegalArgumentException("shooter power of " + power + " cannot be less than min power of " + shooterParams.minPower);
        leftMotor.setPower(power);
        rightMotor.setPower(power);
    }
    public double getAvgMotorSpeed() {
        double leftSpeed = Math.abs(leftMotor.getVelocity(AngleUnit.DEGREES));
        double rightSpeed = Math.abs(rightMotor.getVelocity(AngleUnit.DEGREES));
        return (leftSpeed + rightSpeed) / 2;
    }
    public double getAvgServoPosition() {
        return (leftServo.getPosition() + rightServo.getPosition()) / 2.;
    }
    private void setLerpedServoPositions(double pos) {
        double lerpedValue = MathUtils.lerp(hoodDownPosition, hoodUpPosition, pos);
        leftServo.setPosition(lerpedValue);
        rightServo.setPosition(lerpedValue);
    }
    private void setRawServoPositions(double pos) {
        pos = Range.clip(pos, hoodUpPosition, hoodDownPosition);
        leftServo.setPosition(pos);
        rightServo.setPosition(pos);
    }

    @Override
    public void printInfo() {
        telemetry.addLine("===SHOOTER===");
        telemetry.addData("state", state);
        telemetry.addLine();
        telemetry.addData("pid motor target", speedPidf.getTarget());
        telemetry.addData("pid motor power", pidMotorPower);
        telemetry.addData("target speed", shooterParams.nearZoneTargetSpeed);
        telemetry.addData("actual avg speed", MathUtils.format3(getAvgMotorSpeed()));
        telemetry.addData("left speed", leftMotor.getVelocity(AngleUnit.DEGREES));
        telemetry.addData("right speed", rightMotor.getVelocity(AngleUnit.DEGREES));
        telemetry.addLine();
        telemetry.addData("left motor power", MathUtils.format3(leftMotor.getPower()));
        telemetry.addData("right motor power", MathUtils.format3(rightMotor.getPower()));
        telemetry.addData("left servo position", MathUtils.format3(leftServo.getPosition()));
        telemetry.addData("right servo position", MathUtils.format3(rightServo.getPosition()));
    }

    private Command shooterTrackerCommand() {
        return new Command() {
            private int num = 0;
            private double lastTime = 0;
            @Override
            public Set<com.arcrobotics.ftclib.command.Subsystem> getRequirements() {
                return Collections.emptySet();
            }
            @Override
            public void initialize() {
                ShooterSpeedRecorder.data = new double[ShooterSpeedRecorder.recordAmount][ShooterSpeedRecorder.numDataEntries];
            }
            @Override
            public void execute() {
                if (timer.milliseconds() - lastTime > ShooterSpeedRecorder.recordIntervalMs && num < ShooterSpeedRecorder.recordAmount) {
                    lastTime = timer.milliseconds();
                    ShooterSpeedRecorder.data[num][0] = timer.milliseconds();
                    ShooterSpeedRecorder.data[num][1] = getAvgMotorSpeed();
                    ShooterSpeedRecorder.data[num][2] = pidMotorPower;
                    ShooterSpeedRecorder.data[num][3] = getAvgServoPosition();
                    num++;
                }
            }
            @Override
            public boolean isFinished() {
                return state == State.TRACK_PASSIVE_SPEED || num >= ShooterSpeedRecorder.recordAmount;
            }
        };
    }

    public double getDriveForwardsShooterSpeed() {
        return shooterParams.nearZoneDriveForwardShooterSpeed;
    }
    public double getDriveForwardsDrivetrainSpeed() {
        return shooterParams.nearZoneDriveForwardDrivetrainSpeed;
    }
}
