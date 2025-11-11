package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.Command;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.opModesCompetition.tele.Keybinds;
import org.firstinspires.ftc.teamcode.utils.generalOpModes.Alliance;
import org.firstinspires.ftc.teamcode.utils.generalOpModes.OpmodeType;
import org.firstinspires.ftc.teamcode.utils.misc.Field;
import org.firstinspires.ftc.teamcode.utils.math.MathUtils;
import org.firstinspires.ftc.teamcode.utils.math.PIDFController;
import org.firstinspires.ftc.teamcode.utils.stateManagement.Subsystem;

import java.util.Collections;
import java.util.Set;

@Config
public class Drivetrain extends Subsystem {
    public static class HeadingLockParams {
        public boolean testing = false;
        public double bigKp = -0.7, bigKd = 0.01, smallKp = -2, smallKd = 0;
        public double smallHeadingThresholdDeg = 10;
        public double headingTolDeg = 1;
    }
    public static class TeleDriveParams {
        public double lateralScaling = 1, axialScaling = 1, headingScaling = 1;
        public double minTeleLateralSpeed = 0.4, minTeleAxialSpeed = 0.4, minTeleTurnSpeed = 0.05, maxTeleTurnSpeed = 1, correctiveStrafeHeadingMultiplier = -0.15, minLateralSpeedForStrafeCorrection = 0.7;
        public double intakeLateralScale = 1, intakeAxialScale = 1, intakeHeadingScale = 0.7;
        public double parkLateralScale = 0.5, parkAxialScale = 0.5, parkHeadingScale = 0.4;
        public double parkPreciseLateralTol = 3, parkPreciseAxialTol =  1.5, parkPreciseHeadingDegTol = 2;
        public double parkBasicLateralTol = 3, parkBasicAxialTol = 2.5, parkBasicHeadingDegTol = 4;
        public double redResetLateralPower = -0.7, redResetAxialPower = 0.62, blueResetLateralPower = 0.7, blueResetAxialPower = 0.62;
    }

    public static double fineAdjustPower = 0.12;
    public static HeadingLockParams headingLockParams = new HeadingLockParams();
    public static TeleDriveParams teleDriveParams = new TeleDriveParams();
    public static long resetPositionTimeDelayMs = 400, resetPositionDelayAfterTimeMs = 500;
    public enum State {
        AUTONOMOUS,
        TELE_DRIVE,
        TELE_SLOW_DRIVE
    }
    private State state;
    private DcMotorEx fr, fl, br, bl;
    private final PIDFController headingLockRadPid;
    private double slowDriveLateralScale, slowDriveAxialScale, slowDriveHeadingScale;
    private final ElapsedTime stateTimer;

    public Drivetrain(Hardware hardware, Telemetry telemetry, OpmodeType opmodeType) {
        super(hardware, telemetry);
        stateTimer = new ElapsedTime();
        stateTimer.reset();
        this.state = opmodeType == OpmodeType.TELE ? State.TELE_DRIVE : State.AUTONOMOUS;
        headingLockRadPid = new PIDFController(headingLockParams.bigKp, 0, headingLockParams.bigKd, 0);
        headingLockRadPid.setTarget(0);
        headingLockRadPid.setOutputBounds(-1, 1);
        setIntakeSlowDriveScale();
    }

    public boolean hasParkSlowDriveScale() {
        return Math.abs(slowDriveLateralScale - teleDriveParams.parkLateralScale) < 0.01;
    }
    public void setParkSlowDriveScale() {
        slowDriveLateralScale = teleDriveParams.parkLateralScale;
        slowDriveAxialScale = teleDriveParams.parkAxialScale;
        slowDriveHeadingScale = teleDriveParams.parkHeadingScale;
    }
    public void setIntakeSlowDriveScale() {
        slowDriveLateralScale = teleDriveParams.intakeLateralScale;
        slowDriveAxialScale = teleDriveParams.intakeAxialScale;
        slowDriveHeadingScale = teleDriveParams.intakeHeadingScale;
    }
    public boolean inPreciseParkTolerance() {
        Pose2d pose = robot.pinpoint.pose();
        return Math.abs(pose.position.x - getParkX()) < teleDriveParams.parkPreciseLateralTol &&
                Math.abs(pose.position.y - Field.parkY) < teleDriveParams.parkPreciseAxialTol &&
                Math.abs(Math.toDegrees(pose.heading.toDouble()) - Field.parkADeg) < teleDriveParams.parkPreciseHeadingDegTol;
    }
    public boolean inBasicParkTolerance() {
        Pose2d pose = robot.pinpoint.pose();
        return Math.abs(pose.position.x - getParkX()) < teleDriveParams.parkBasicLateralTol &&
                Math.abs(pose.position.y - Field.parkY) < teleDriveParams.parkBasicAxialTol &&
                Math.abs(Math.toDegrees(pose.heading.toDouble()) - Field.parkADeg) < teleDriveParams.parkBasicHeadingDegTol;
    }

    private double getParkX() {
        return robot.alliance == Alliance.BLUE ? Field.blueParkX : Field.redParkX;
    }

    public void setState(State state) {
        this.state = state;
    }

    @Override
    public void declareHardware() {
        fr = hardware.getFRDriveMotor();
        fl = hardware.getFLDriveMotor();
        br = hardware.getBRDriveMotor();
        bl = hardware.getBLDriveMotor();
    }

    @Override
    public void updateState() {
        if (keybinds.check(Keybinds.D2Trigger.APPLY_PINPOINT_RESET_POWERS)) {
            if (robot.alliance == Alliance.BLUE)
                setDrivePowers(teleDriveParams.blueResetLateralPower, teleDriveParams.blueResetAxialPower, 0);
            else
                setDrivePowers(teleDriveParams.redResetLateralPower, teleDriveParams.redResetAxialPower, 0);
            return;
        }
        if (keybinds.check(Keybinds.D2Trigger.RESET_PINPOINT)) {
            setMotorPowers(0, 0, 0, 0);
            double x = robot.alliance == Alliance.BLUE ? Field.blueResetX : Field.redResetX;
            double angleRad = Math.toRadians(robot.alliance == Alliance.BLUE ? Field.blueResetADeg : Field.redResetADeg);
            Pose2d initialPose = new Pose2d(x, Field.resetY, angleRad);

            robot.light.setColor(RGBLight.params.red);
            try {
                Thread.sleep(resetPositionTimeDelayMs);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            robot.light.setColor(RGBLight.params.purple);
            robot.pinpoint.resetPoseTo(initialPose, resetPositionDelayAfterTimeMs);
            robot.light.startUpdatingSelfAgain();
            return;
        }

        if (g2.isDpadRightPressed()) {
            setDrivePowers(0, 0, -fineAdjustPower);
            return;
        }
        if (g2.isDpadLeftPressed()) {
            setDrivePowers(0, 0, fineAdjustPower);
            return;
        }

        double[] linearPowers;
        switch (state) {
            case TELE_DRIVE:
                linearPowers = calculateLinearPowersWithGamepadInput();
                // auto aim
                if (keybinds.check(Keybinds.D1Trigger.AUTO_AIM)) {
                    double headingPower = getHeadingLockPower(getTargetShootHeadingRad());
                    if (headingLockParams.testing)
                        setDrivePowers(0, 0, 0);
                    else
                        setDrivePowers(0, 0, headingPower);
                }
                // normal driving
                else {
                    // correct heading power based off normal lateral speed
                    double headingPower = calculateHeadingPowerWithGamepadInput(linearPowers, false);
                    setDrivePowers(linearPowers[0], linearPowers[1], headingPower);
                }
                break;
            case TELE_SLOW_DRIVE:
                if (keybinds.check(Keybinds.D1Trigger.AUTO_AIM) && !hasParkSlowDriveScale()) {
                    setState(State.TELE_DRIVE);
                    break;
                }

                linearPowers = calculateLinearPowersWithGamepadInput();
                linearPowers[0] *= slowDriveLateralScale;
                linearPowers[1] *= slowDriveAxialScale;
                // correct heading power based off normal lateral speed
                double headingPower = calculateHeadingPowerWithGamepadInput(linearPowers, false);

                // override heading with heading lock
                if (keybinds.check(Keybinds.D1Trigger.AUTO_AIM)) {
                    linearPowers[0] = 0;
                    linearPowers[1] = 0;
                    headingPower = getHeadingLockPower(Math.toRadians(Field.parkADeg));
                }

                headingPower *= slowDriveHeadingScale;
                setDrivePowers(linearPowers[0], linearPowers[1], headingPower);

                break;
            case AUTONOMOUS:
                // do nothing
                break;
        }
    }

    public double[] calculateLinearPowersWithGamepadInput() {
        double xSign = Math.signum(g1.getLeftStickX());
        double ySign = Math.signum(g1.getLeftStickY());
        double lateral = xSign * Math.max(teleDriveParams.minTeleLateralSpeed, Math.abs(Math.pow(g1.getLeftStickX(), teleDriveParams.lateralScaling)));
        double axial = ySign * -Math.max(teleDriveParams.minTeleAxialSpeed, Math.abs(Math.pow(g1.getLeftStickY(), teleDriveParams.axialScaling)));
        return new double[]{ lateral, axial };
    }
    public double calculateHeadingPowerWithGamepadInput(double[] powers, boolean correctStrafe) {
        double headingSign = Math.signum(g1.getRightStickX());
        double heading = headingSign * -Math.max(teleDriveParams.minTeleTurnSpeed, Math.abs(Math.pow(g1.getRightStickX(), teleDriveParams.headingScaling)));
        headingSign = Math.signum(heading);
        heading = headingSign * Math.min(Math.abs(heading), teleDriveParams.maxTeleTurnSpeed);
        double correctiveStrafeHeading = 0;
        if (correctStrafe && Math.abs(powers[0]) > teleDriveParams.minLateralSpeedForStrafeCorrection && powers[1] == 0)
            correctiveStrafeHeading = powers[0] * teleDriveParams.correctiveStrafeHeadingMultiplier;

        return Range.clip(heading + correctiveStrafeHeading, -1, 1);
    }

    @Override
    public void printInfo() {
        telemetry.addLine("===DRIVETRAIN===");
        telemetry.addData("state", state);
        printMotorPowers();
    }

    public State getState() {
        return state;
    }

    public void setDrivePowers(double lateralPower, double axialPower, double headingPower) {
        double addValue = Math.round((100 * (axialPower * Math.abs(axialPower) + lateralPower * Math.abs(lateralPower)))) / 100.;
        double subtractValue = Math.round((100 * (axialPower * Math.abs(axialPower) - lateralPower * Math.abs(lateralPower)))) / 100.;
        setMotorPowers((addValue - headingPower), (subtractValue + headingPower), (subtractValue - headingPower), (addValue + headingPower));
    }
    public void setMotorPowers(double flPower, double frPower, double blPower, double brPower) {
        fl.setPower(flPower);
        fr.setPower(frPower);
        bl.setPower(blPower);
        br.setPower(brPower);
    }

    public void printMotorPowers() {
        telemetry.addData("dt powers (FL FR BL BR)", MathUtils.format3(fl.getPower()) + ", " + MathUtils.format3(fr.getPower()) + ", " + MathUtils.format3(bl.getPower()) + ", " + MathUtils.format3(br.getPower()));
    }
    private double getHeadingLockPower(double targetHeadingRad) {
        double headingErrorRad = targetHeadingRad - robot.pinpoint.pose().heading.toDouble();
        double absHeadingErrorRad = Math.abs(headingErrorRad);
        if (absHeadingErrorRad < Math.toRadians(headingLockParams.headingTolDeg))
            return 0;
        boolean flipHeadingPower = absHeadingErrorRad > Math.PI;
        if (flipHeadingPower)
            headingErrorRad = Math.signum(headingErrorRad) * (Math.PI * 2 - absHeadingErrorRad);

        if (absHeadingErrorRad > Math.toRadians(headingLockParams.smallHeadingThresholdDeg))
            headingLockRadPid.setPIDValues(headingLockParams.bigKp, 0, headingLockParams.bigKd, 0);
        else
            headingLockRadPid.setPIDValues(headingLockParams.smallKp, 0, headingLockParams.smallKd, 0);

        double headingPower = headingLockRadPid.update(headingErrorRad);
        if (flipHeadingPower)
            headingPower *= -1;

//        telemetry.addData("flip power", flipHeadingPower);
//        telemetry.addData("kp", headingLockRadPid.kP);
//        telemetry.addData("error", Math.toDegrees(headingError));
//        telemetry.addData("power", headingPower);

        return headingPower;
    }
    private double getTargetShootHeadingRad() {
        double nextX = robot.pinpoint.pose().position.x + robot.pinpoint.driver.getVelX(DistanceUnit.INCH);
        double nextY = robot.pinpoint.pose().position.y + robot.pinpoint.driver.getVelY(DistanceUnit.INCH);
        double goalX = robot.alliance == Alliance.RED ? Field.redGoalX : Field.blueGoalX;
        double desiredAngle = Math.atan2(Field.goalY - nextY, goalX - nextX);

        telemetry.addData("desired angle", Math.round(Math.toDegrees(desiredAngle)));
        return desiredAngle;
    }
    public boolean headingWithinShootingTolerance() {
        return Math.abs(getTargetShootHeadingRad() - robot.pinpoint.pose().heading.toDouble()) < Math.toRadians(headingLockParams.headingTolDeg);
    }
    public Command headingLockCommand(double maxTime, double headingOffsetDeg) {
        return new Command() {
            private final ElapsedTime timer = new ElapsedTime();
            @Override
            public void execute() {
                setDrivePowers(0, 0, getHeadingLockPower(getTargetShootHeadingRad() + Math.toRadians(headingOffsetDeg)));
            }
            @Override
            public boolean isFinished() {
                return headingWithinShootingTolerance() || timer.seconds() >= maxTime;
            }
            @Override
            public void end(boolean interrupted) {
                setDrivePowers(0, 0, 0);
            }
            @Override
            public Set<com.arcrobotics.ftclib.command.Subsystem> getRequirements() {
                return Collections.emptySet();
            }
        };
    }
}
