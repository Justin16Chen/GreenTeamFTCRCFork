package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
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
        public double slowAxialSpeed = 0.7, slowLateralSpeed = 0.85;
        public double kp = 0.02, ki = 0, kd = 0, kf = 0.;
        public double headingTol = 1;
    }
    public static CorrectiveDriveParams correctiveDriveParams = new CorrectiveDriveParams();
    public static HeadingLockParams headingLockParams = new HeadingLockParams();
    public enum State {
        AUTONOMOUS,
        TELE_DRIVE
    }
    public static double lateralScaling = 3, axialScaling = 3, headingScaling = 3;
    public static double minTurnSpeed = 0.3, correctiveStrafeHeadingMultiplier = 0.15;
    private final State state;
    private DcMotorEx fr, fl, br, bl;
    private final PIDFController headingLockPid;

    public Drivetrain(Hardware hardware, Telemetry telemetry, OpmodeType opmodeType) {
        super(hardware, telemetry);
        this.state = opmodeType == OpmodeType.TELE ? State.TELE_DRIVE : State.AUTONOMOUS;
        headingLockPid = new PIDFController(headingLockParams.kp, headingLockParams.ki, headingLockParams.kd, headingLockParams.kf);
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
        boolean flipHeadingPower = absHeadingError > Math.PI;
        if (flipHeadingPower)
            headingError = Math.signum(headingError) * (360 - absHeadingError);

        double headingPower = headingLockPid.update(headingError);
        if (flipHeadingPower)
            headingPower *= -1;

        return headingPower;
    }
    private double getHeadingShootError() {
        double nextX = robot.pinpoint.pose().position.x + robot.pinpoint.driver.getVelX(DistanceUnit.INCH);
        double nextY = robot.pinpoint.pose().position.y + robot.pinpoint.driver.getVelY(DistanceUnit.INCH);
        double goalX = robot.alliance == Alliance.RED ? Field.redGoalX : Field.blueGoalX;
        double desiredAngle = Math.atan2(Field.goalY - nextY, goalX - nextX);

        return desiredAngle - robot.pinpoint.pose().heading.toDouble();
    }
    public boolean headingWithinShootingTolerance() {
        return getHeadingShootError() < headingLockParams.headingTol;
    }
    private void driveToDesiredPose() {
        double x = robot.alliance == Alliance.RED ? Field.redNearZoneShootX : Field.blueNearZoneShootX;
        double angle = robot.alliance == Alliance.RED ? Field.redNearZoneShootAngle : Field.blueNearZoneShootAngle;
        Pose2d desiredPose = new Pose2d(x, Field.nearZoneShootY, angle);

        // create drive path
        Tolerance tolerance = new Tolerance(correctiveDriveParams.distTol, Math.toRadians(correctiveDriveParams.headingTol));
        PathParams pathParams = new PathParams(correctiveDriveParams.drivePIDs);
        pathParams.minSpeed = correctiveDriveParams.minSpeed;
        pathParams.maxSpeed = correctiveDriveParams.maxSpeed;
        pathParams.maxHeadingSpeed = correctiveDriveParams.maxHeadingSpeed;
        Waypoint waypoint = new Waypoint(desiredPose, tolerance, pathParams);
        DrivePath correctiveDrive = new DrivePath(this, robot.pinpoint, waypoint, telemetry);
        correctiveDrive.schedule();

        robot.pinpoint.printInfo();
        telemetry.addLine();
        telemetry.addData("desired pose", MathUtils.format3(desiredPose.position.x) + ", " + MathUtils.format3(desiredPose.position.y) + ", " + MathUtils.format3(Math.toDegrees(desiredPose.heading.toDouble())));
        telemetry.update();

        // old math
//        Vector2d goalPosition = new Vector2d(robot.alliance == Alliance.BLUE ? Field.blueGoalX : Field.redGoalX, Field.goalY);
//        double rx = robot.pinpoint.pose().position.x, ry = robot.pinpoint.pose().position.y;
//        Vector2d goalToRobot = new Vector2d(rx - goalPosition.x, ry - Field.goalY);
//        goalToRobot = goalToRobot.div(Math.hypot(goalToRobot.x, goalToRobot.y));
//        Vector2d desiredPosition = goalPosition.plus(goalToRobot.times(Shooter.correctiveDriveParams.desiredNearShootDistance));
//        double desiredHeading = Math.atan2(Field.goalY - desiredPosition.y, goalPosition.y - desiredPosition.y);
//
//        telemetry.addData("goalX", goalPosition.x);
//        telemetry.addData("goalY", goalPosition.y);
//        telemetry.addData("goal to robot angle", MathUtils.format3(Math.toDegrees(Math.atan2(goalToRobot.y, goalToRobot.x))));
//
//        return new Pose2d(desiredPosition, desiredHeading);
    }
}
