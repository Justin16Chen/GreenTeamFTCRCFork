package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.Command;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.opModesCompetition.tele.Keybinds;
import org.firstinspires.ftc.teamcode.utils.generalOpModes.Alliance;
import org.firstinspires.ftc.teamcode.utils.generalOpModes.OpmodeType;
import org.firstinspires.ftc.teamcode.utils.misc.Field;
import org.firstinspires.ftc.teamcode.utils.misc.MathUtils;
import org.firstinspires.ftc.teamcode.utils.misc.PIDFController;
import org.firstinspires.ftc.teamcode.utils.stateManagement.Subsystem;

import java.util.Collections;
import java.util.Set;

@Config
public class Drivetrain extends Subsystem {
    public static class HeadingLockParams {
        public boolean testing = false;
        public double slowAxialSpeed = 0.7, slowLateralSpeed = 0.85;
        public double bigKp = -0.7, bigKd = 0.01, smallKp = -2, smallKd = 0;
        public double smallHeadingThresholdDeg = 10;
        public double headingTolDeg = 1;
    }
    public static HeadingLockParams headingLockParams = new HeadingLockParams();
    public enum State {
        AUTONOMOUS,
        TELE_DRIVE,
        TELE_SLOW_DRIVE
    }
    public static double lateralScaling = 2, axialScaling = 2, headingScaling = 2;
    public static double minTeleLateralSpeed = 0.4, minTeleAxialSpeed = 0.4, minTeleTurnSpeed = 0.1, maxTeleTurnSpeed = 1, correctiveStrafeHeadingMultiplier = -0.15, minLateralSpeedForStrafeCorrection = 0.7;
    public static double intakeLateralScale = 1, intakeAxialScale = 1, intakeHeadingScale = 0.7;
    public static double parkLateralScale = 0.5, parkAxialScale = 0.5, parkHeadingScale = 0.4;
    public static double parkPreciseLateralTol = 3, parkPreciseAxialTol =  1.5, parkPreciseHeadingDegTol = 2;
    public static double parkBasicLateralTol = 3, parkBasicAxialTol = 2.5, parkBasicHeadingDegTol = 4;
//    public static double redResetLateralPower = -0.5, redResetY
    private State state;
    private DcMotorEx fr, fl, br, bl;
    private final PIDFController headingLockRadPid;
    private double slowDriveLateralScale, slowDriveAxialScale, slowDriveHeadingScale;

    public Drivetrain(Hardware hardware, Telemetry telemetry, OpmodeType opmodeType) {
        super(hardware, telemetry);
        this.state = opmodeType == OpmodeType.TELE ? State.TELE_DRIVE : State.AUTONOMOUS;
        headingLockRadPid = new PIDFController(headingLockParams.bigKp, 0, headingLockParams.bigKd, 0);
        headingLockRadPid.setTarget(0);
        headingLockRadPid.setOutputBounds(-1, 1);
        setIntakeSlowDriveScale();
    }

    public boolean hasParkSlowDriveScale() {
        return Math.abs(slowDriveLateralScale - parkLateralScale) < 0.01;
    }
    public void setParkSlowDriveScale() {
        slowDriveLateralScale = parkLateralScale;
        slowDriveAxialScale = parkAxialScale;
        slowDriveHeadingScale = parkHeadingScale;
    }
    public void setIntakeSlowDriveScale() {
        slowDriveLateralScale = intakeLateralScale;
        slowDriveAxialScale = intakeAxialScale;
        slowDriveHeadingScale = intakeHeadingScale;
    }
    public boolean inPreciseParkTolerance() {
        Pose2d pose = robot.pinpoint.pose();
        return Math.abs(pose.position.x - getParkX()) < parkPreciseLateralTol &&
                Math.abs(pose.position.y - Field.parkY) < parkPreciseAxialTol &&
                Math.abs(Math.toDegrees(pose.heading.toDouble()) - Field.parkADeg) < parkPreciseHeadingDegTol;
    }
    public boolean inBasicParkTolerance() {
        Pose2d pose = robot.pinpoint.pose();
        return Math.abs(pose.position.x - getParkX()) < parkBasicLateralTol &&
                Math.abs(pose.position.y - Field.parkY) < parkBasicAxialTol &&
                Math.abs(Math.toDegrees(pose.heading.toDouble()) - Field.parkADeg) < parkBasicHeadingDegTol;
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

        }
        if (keybinds.check(Keybinds.D2Trigger.RESET_PINPOINT_POSE)) {
            setMotorPowers(0, 0, 0, 0);
            double x = robot.alliance == Alliance.BLUE ? Field.blueResetX : Field.redResetX;
            double angleRad = Math.toRadians(robot.alliance == Alliance.BLUE ? Field.blueResetADeg : Field.redResetADeg);
            Pose2d initialPose = new Pose2d(x, Field.resetY, angleRad);
            robot.pinpoint.setInitialPose(initialPose);
        }

        double[] linearPowers;
        switch (state) {
            case TELE_DRIVE:
                // calculate lateral and axial powers
                linearPowers = calculateLinearPowersWithGamepadInput();

                if (keybinds.check(Keybinds.D1Trigger.AUTO_AIM)) {
                    // slow down for increased accuracy
                    linearPowers[0] = headingLockParams.slowLateralSpeed * linearPowers[0];
                    linearPowers[1] = headingLockParams.slowAxialSpeed * linearPowers[1];

                    // correct heading power based off of new slowed-down lateral speed
                    double headingPower = getHeadingLockPower(getTargetShootHeadingRad());
                    if (Math.abs(linearPowers[0]) > minLateralSpeedForStrafeCorrection)
                        headingPower += linearPowers[0] * correctiveStrafeHeadingMultiplier;
                    headingPower = Range.clip(headingPower, -1, 1);

                    if (headingLockParams.testing)
                        setDrivePowers(0, 0, 0);
                    else
                        setDrivePowers(linearPowers[0], linearPowers[1], headingPower);
                }
                else {
                    // correct heading power based off normal lateral speed
                    double headingPower = calculateHeadingPowerWithGamepadInput(linearPowers);
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
                double headingPower = calculateHeadingPowerWithGamepadInput(linearPowers);

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
        double lateral = xSign * Math.max(minTeleLateralSpeed, Math.abs(Math.pow(g1.getLeftStickX(), lateralScaling)));
        double axial = ySign * -Math.max(minTeleAxialSpeed, Math.abs(Math.pow(g1.getLeftStickY(), axialScaling)));
        return new double[]{ lateral, axial };
    }
    public double calculateHeadingPowerWithGamepadInput(double[] powers) {
        double headingSign = Math.signum(g1.getRightStickX());
        double heading = headingSign * -Math.max(minTeleTurnSpeed, Math.abs(Math.pow(g1.getRightStickX(), headingScaling)));
        headingSign = Math.signum(heading);
        heading = headingSign * Math.min(Math.abs(heading), maxTeleTurnSpeed);
        double correctiveStrafeHeading = 0;
        if (Math.abs(powers[0]) > minLateralSpeedForStrafeCorrection && powers[1] == 0)
            correctiveStrafeHeading = powers[0] * correctiveStrafeHeadingMultiplier;

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
        double headingError = targetHeadingRad - robot.pinpoint.pose().heading.toDouble();
        double absHeadingError = Math.abs(headingError);
        if (absHeadingError < Math.toRadians(headingLockParams.headingTolDeg))
            return 0;
        boolean flipHeadingPower = absHeadingError > Math.PI;
        if (flipHeadingPower)
            headingError = Math.signum(headingError) * (Math.PI * 2 - absHeadingError);

        if (absHeadingError > Math.toRadians(headingLockParams.smallHeadingThresholdDeg))
            headingLockRadPid.setPIDValues(headingLockParams.bigKp, 0, headingLockParams.bigKd, 0);
        else
            headingLockRadPid.setPIDValues(headingLockParams.smallKp, 0, headingLockParams.smallKd, 0);

        double headingPower = headingLockRadPid.update(headingError);
        if (flipHeadingPower)
            headingPower *= -1;

        telemetry.addData("flip power", flipHeadingPower);
        telemetry.addData("kp", headingLockRadPid.kP);
        telemetry.addData("error", Math.toDegrees(headingError));
        telemetry.addData("power", headingPower);

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
    public Command headingLockCommand() {
        return new Command() {
            @Override
            public void execute() {
                setDrivePowers(0, 0, getHeadingLockPower(getTargetShootHeadingRad()));
            }
            @Override
            public boolean isFinished() {
                return headingWithinShootingTolerance();
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
