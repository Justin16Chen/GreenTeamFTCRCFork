package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.opModesCompetition.tele.Keybinds;
import org.firstinspires.ftc.teamcode.utils.generalOpModes.OpmodeType;
import org.firstinspires.ftc.teamcode.utils.misc.MathUtils;
import org.firstinspires.ftc.teamcode.utils.misc.MotorCurrentTracker;
import org.firstinspires.ftc.teamcode.utils.stateManagement.Subsystem;

@Config
public class Intake extends Subsystem {
    public static double collectPower = 0.99, outtakePower = -0.8, passivePower = 0.5, weakPassivePower = 0.4, feedShooterNearZonePower = 0.6, feedShooterFarPower = 0.5, feedShooterNearAutoSlow = 0.4;
    public static double minPreciseFeedShooterTime = 3;
    public static double preciseTrackingValidationFrames = 10;
    public static int maxNormalCurrent = 6200, abnormalCurrentValidationFrames = 2, abnormalCurrentSafetyFrames = 1;

    public void setUseAutoSlowFeedShooterPower(boolean b) {
        this.useAutoSlowFeedShooterPower = true;
    }

    public enum State {
        ON, OFF, PASSIVE_INTAKE, FEED_SHOOTER_PRECISE, FEED_SHOOTER_TELE_TOGGLE
    }

    private State state;
    private DcMotorEx motor;
    private final ElapsedTime stateTimer;
    private MotorCurrentTracker currentTracker;

    private int numConsecutiveValidatedFrames;
    private int officialNumBalls, unofficalNumBalls;
    private boolean useAutoSlowFeedShooterPower = false;
    public Intake(Hardware hardware, Telemetry telemetry) {
        super(hardware, telemetry);
        state = State.OFF;
        stateTimer = new ElapsedTime();
        stateTimer.reset();
    }

    @Override
    public void declareHardware() {
        motor = hardware.getIntakeMotor();
        currentTracker = new MotorCurrentTracker(motor, maxNormalCurrent, abnormalCurrentValidationFrames, abnormalCurrentSafetyFrames);
    }

    @Override
    public void updateState() {

        switch (state) {
            case OFF:
                if (keybinds.check(Keybinds.D1Trigger.TOGGLE_INTAKE)) {
                    setState(State.ON);
                    break;
                }
                if (keybinds.check(Keybinds.D1Trigger.MANUAL_OUTTAKE))
                    motor.setPower(outtakePower);
                else
                    motor.setPower(0);
                break;
            case ON:
                if (keybinds.check(Keybinds.D1Trigger.TOGGLE_INTAKE)) {
                    setState(State.OFF);
                    break;
                }
                if (keybinds.check(Keybinds.D2Trigger.TOGGLE_PASSIVE_INTAKE)) {
                    setState(State.PASSIVE_INTAKE);
                    break;
                }

                currentTracker.updateCurrentTracking();

//                if (keybinds.check(Keybinds.D1Trigger.MANUAL_OUTTAKE) || currentTracker.hasValidatedAbnormalCurrent())
                if (keybinds.check(Keybinds.D1Trigger.MANUAL_OUTTAKE))
                    motor.setPower(outtakePower);
                else
                    motor.setPower(collectPower);

                break;
            case PASSIVE_INTAKE:
                if (keybinds.check(Keybinds.D2Trigger.TOGGLE_PASSIVE_INTAKE)) {
                    setState(State.OFF);
                    break;
                }
                if (keybinds.check(Keybinds.D1Trigger.TOGGLE_INTAKE)) {
                    setState(State.ON);
                    break;
                }
                if (keybinds.check(Keybinds.D1Trigger.MANUAL_OUTTAKE))
                    motor.setPower(outtakePower);
                else if (robot.flipper.isMoving())
                    motor.setPower(weakPassivePower);
                else
                    motor.setPower(passivePower);
                break;
            case FEED_SHOOTER_TELE_TOGGLE:
                if (!keybinds.check(Keybinds.D1Trigger.CONTINUE_SHOOTING)) {
                    setState(State.OFF);
                    break;
                }

                motor.setPower(getFeedShooterPower());
                break;
            case FEED_SHOOTER_PRECISE:
                updateNumBalls();
                if (officialNumBalls == 0 && stateTimer.seconds() > minPreciseFeedShooterTime) {
                    setState(State.OFF);
                    break;
                }

                if (useAutoSlowFeedShooterPower && robot.shooter.getZone() == Shooter.Zone.NEAR)
                    motor.setPower(feedShooterNearAutoSlow);
                else
                    motor.setPower(getFeedShooterPower());
                break;
        }
    }
    public int getOfficialNumBalls() {
        return officialNumBalls;
    }

    public State getState() { return state; }
    public void setState(State newState) {
        if (state == newState)
            return;

        state = newState;
        stateTimer.reset();

        if (state == State.ON) {
            motor.setPower(collectPower);
            turnOnSensors(false);
            if (robot.opmodeType == OpmodeType.TELE) {
                robot.drivetrain.setState(Drivetrain.State.TELE_SLOW_DRIVE);
                robot.drivetrain.setIntakeSlowDriveScale();
            }
        }
        else if (state == State.OFF) {
            motor.setPower(0);
            turnOnSensors(false);
            if (robot.opmodeType == OpmodeType.TELE && robot.drivetrain.getState() == Drivetrain.State.TELE_SLOW_DRIVE)
                robot.drivetrain.setState(Drivetrain.State.TELE_DRIVE);
        }
        else if (state == State.PASSIVE_INTAKE) {
            motor.setPower(passivePower);
            turnOnSensors(false);
            if (robot.opmodeType == OpmodeType.TELE && robot.drivetrain.getState() == Drivetrain.State.TELE_SLOW_DRIVE)
                robot.drivetrain.setState(Drivetrain.State.TELE_DRIVE);
        }
        else if (state == State.FEED_SHOOTER_PRECISE) {
            officialNumBalls = -1;
            motor.setPower(feedShooterNearZonePower);
            turnOnSensors(true);
        }
    }

    private void turnOnSensors(boolean turnedOn) {
        for (BallColorSensor sensor : robot.colorSensors)
            sensor.setTurnedOn(turnedOn);
    }

    public void printInfo() {
        telemetry.addLine("===INTAKE===");
        telemetry.addData("state", state);
        telemetry.addData("keybind activated", keybinds.check(Keybinds.D1Trigger.TOGGLE_INTAKE));
        telemetry.addData("motor power", MathUtils.format3(motor.getPower()));
        telemetry.addData("motor current", MathUtils.format3(motor.getCurrent(CurrentUnit.MILLIAMPS)));
    }
    private int getCurFrameNumBalls() {
        int curFrameNumBalls = 0;
        for (BallColorSensor sensor : robot.colorSensors)
            if (sensor.seesBallFromLatestCache())
                curFrameNumBalls++;
        return curFrameNumBalls;
    }

    private void updateNumBalls() {
        // track number of frames that a consistent reading has happened
        int newNumBalls = getCurFrameNumBalls();
        if (newNumBalls == unofficalNumBalls)
            numConsecutiveValidatedFrames++;
        else
            numConsecutiveValidatedFrames = 0;

        // update latest unofficial value
        unofficalNumBalls = newNumBalls;

        // update official value
        if (numConsecutiveValidatedFrames >= preciseTrackingValidationFrames ||  // validated value
                officialNumBalls == 0 && unofficalNumBalls > 0) // better to overestimate than underestimate
            officialNumBalls = unofficalNumBalls;
    }
    private double getFeedShooterPower() {
        return robot.shooter.getZone() == Shooter.Zone.NEAR ? feedShooterNearZonePower : feedShooterFarPower;
    }
}
