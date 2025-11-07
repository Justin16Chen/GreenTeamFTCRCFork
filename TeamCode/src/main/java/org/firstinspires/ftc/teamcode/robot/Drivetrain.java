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
import org.firstinspires.ftc.teamcode.utils.pidDrive.DrivePath;
import org.firstinspires.ftc.teamcode.utils.pidDrive.PathParams;
import org.firstinspires.ftc.teamcode.utils.pidDrive.Tolerance;
import org.firstinspires.ftc.teamcode.utils.pidDrive.Waypoint;
import org.firstinspires.ftc.teamcode.utils.stateManagement.Subsystem;

import java.util.Collections;
import java.util.Set;

@Config
public class Drivetrain extends Subsystem {
    public static class CorrectiveDriveParams {
        public double desiredNearShootDistance = 55.93;
        public double distTol = 1, headingTol = 2;
        public double[] drivePIDs = { 0.07, 0, 0.01, 0.02, 0, 0 };
        public double minSpeed = 0.5;
        public double maxSpeed = 0.8, maxHeadingSpeed = 0.8;
    }
    public static class HeadingLockParams {
        public boolean testing = false;
        public double slowAxialSpeed = 0.7, slowLateralSpeed = 0.85;
        public double bigKp = -0.7, bigKd = 0.01, smallKp = -2, smallKd = 0;
        public double smallHeadingThreshold = 10;
        public double headingTol = 1;
    }
    public static CorrectiveDriveParams correctiveDriveParams = new CorrectiveDriveParams();
    public static HeadingLockParams headingLockParams = new HeadingLockParams();
    public enum State {
        AUTONOMOUS,
        TELE_DRIVE
    }
    public static double lateralScaling = 3, axialScaling = 3, headingScaling = 3;
    public static double minTurnSpeed = 0.3, correctiveStrafeHeadingMultiplier = -0.15;
    private final State state;
    private DcMotorEx fr, fl, br, bl;
    private final PIDFController headingLockPid;

    public Drivetrain(Hardware hardware, Telemetry telemetry, OpmodeType opmodeType) {
        super(hardware, telemetry);
        this.state = opmodeType == OpmodeType.TELE ? State.TELE_DRIVE : State.AUTONOMOUS;
        headingLockPid = new PIDFController(headingLockParams.bigKp, 0, headingLockParams.bigKd, 0);
        headingLockPid.setTarget(0);
        headingLockPid.setOutputBounds(-1, 1);
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
        switch (state) {
            case TELE_DRIVE:
                // calculate lateral and axial powers
                double[] linearPowers = calculateLinearPowersWithGamepadInput();

                if (keybinds.check(Keybinds.D1Trigger.AUTO_AIM)) {
                    // slow down for increased accuracy
                    linearPowers[0] = headingLockParams.slowLateralSpeed * linearPowers[0];
                    linearPowers[1] = headingLockParams.slowAxialSpeed * linearPowers[1];

                    // correct heading power based off of new slowed-down lateral speed
                    double headingPower = getHeadingLockPower();
                    headingPower += linearPowers[0] * correctiveStrafeHeadingMultiplier;
                    headingPower = Range.clip(headingPower, -1, 1);

                    if (headingLockParams.testing)
                        setDrivePowers(0, 0, 0);
                    else
                        setDrivePowers(linearPowers[0], linearPowers[1], headingPower);
                }
                else {
                    // correct heading power based off normal lateral speed
                    double headingPower = calculateHeadingPowerWithGamepadInput(linearPowers[0]);
                    setDrivePowers(linearPowers[0], linearPowers[1], headingPower);
                }
                break;
            case AUTONOMOUS:
                // do nothing
                break;
        }
    }

    public double[] calculateLinearPowersWithGamepadInput() {
        double xSign = Math.signum(g1.getLeftStickX());
        double ySign = Math.signum(g1.getLeftStickY());
        double lateral = xSign * Math.abs(Math.pow(g1.getLeftStickX(), lateralScaling));
        double axial = ySign * -Math.abs(Math.pow(g1.getLeftStickY(), axialScaling));
        return new double[]{ lateral, axial };
    }
    public double calculateHeadingPowerWithGamepadInput(double lateral) {
        double headingSign = Math.signum(g1.getRightStickX());
        double heading = headingSign * -Math.max(minTurnSpeed, Math.abs(Math.pow(g1.getRightStickX(), headingScaling)));
        double correctiveStrafeHeading = lateral * correctiveStrafeHeadingMultiplier;

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
    public void setMotorPowers(double frontLeftPower, double frontRightPower, double backLeftPower, double backRightPower) {
        fl.setPower(frontLeftPower);
        fr.setPower(frontRightPower);
        bl.setPower(backLeftPower);
        br.setPower(backRightPower);
    }

    public void printMotorPowers() {
        telemetry.addData("dt powers (FL FR BL BR)", MathUtils.format3(fl.getPower()) + ", " + MathUtils.format3(fr.getPower()) + ", " + MathUtils.format3(bl.getPower()) + ", " + MathUtils.format3(br.getPower()));
    }
    private double getHeadingLockPower() {
        double headingError = getHeadingShootError();
        double absHeadingError = Math.abs(headingError);
        if (absHeadingError < Math.toRadians(headingLockParams.headingTol))
            return 0;
        boolean flipHeadingPower = absHeadingError > Math.PI;
        if (flipHeadingPower)
            headingError = Math.signum(headingError) * (Math.PI * 2 - absHeadingError);

        if (absHeadingError > Math.toRadians(headingLockParams.smallHeadingThreshold))
            headingLockPid.setPIDValues(headingLockParams.bigKp, 0, headingLockParams.bigKd, 0);
        else
            headingLockPid.setPIDValues(headingLockParams.smallKp, 0, headingLockParams.smallKd, 0);

        double headingPower = headingLockPid.update(headingError);
        if (flipHeadingPower)
            headingPower *= -1;

        telemetry.addData("flip power", flipHeadingPower);
        telemetry.addData("kp", headingLockPid.kP);
        telemetry.addData("error", Math.toDegrees(headingError));
        telemetry.addData("power", headingPower);

        return headingPower;
    }
    private double getHeadingShootError() {
        double nextX = robot.pinpoint.pose().position.x + robot.pinpoint.driver.getVelX(DistanceUnit.INCH);
        double nextY = robot.pinpoint.pose().position.y + robot.pinpoint.driver.getVelY(DistanceUnit.INCH);
        double goalX = robot.alliance == Alliance.RED ? Field.redGoalX : Field.blueGoalX;
        double desiredAngle = Math.atan2(Field.goalY - nextY, goalX - nextX);
        double currentAngle = robot.pinpoint.pose().heading.toDouble();

        telemetry.addData("desired angle", Math.round(Math.toDegrees(desiredAngle)));
        telemetry.addData("current angle", Math.round(Math.toDegrees(currentAngle)));

        return desiredAngle - currentAngle;
    }
    public boolean headingWithinShootingTolerance() {
        return getHeadingShootError() < headingLockParams.headingTol;
    }
    public Command headingLockCommand() {
        return new Command() {
            @Override
            public Set<com.arcrobotics.ftclib.command.Subsystem> getRequirements() {
                return Collections.emptySet();
            }
        };
    }
}
