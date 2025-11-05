package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opModesCompetition.auto.SimpleRedAuto;
import org.firstinspires.ftc.teamcode.opModesCompetition.tele.Keybinds;
import org.firstinspires.ftc.teamcode.utils.generalOpModes.Alliance;
import org.firstinspires.ftc.teamcode.utils.generalOpModes.OpmodeType;
import org.firstinspires.ftc.teamcode.utils.misc.Field;
import org.firstinspires.ftc.teamcode.utils.misc.MathUtils;
import org.firstinspires.ftc.teamcode.utils.pidDrive.DrivePath;
import org.firstinspires.ftc.teamcode.utils.pidDrive.PathParams;
import org.firstinspires.ftc.teamcode.utils.pidDrive.Tolerance;
import org.firstinspires.ftc.teamcode.utils.pidDrive.Waypoint;
import org.firstinspires.ftc.teamcode.utils.stateManagement.Subsystem;

@Config
public class Drivetrain extends Subsystem {

    public enum State {
        AUTONOMOUS,
        TELE_DRIVE,
        CORRECT_TO_SHOOTING_POSITION
    }
    public static double lateralScaling = 3, axialScaling = 3, headingScaling = 3;
    public static double minTurnSpeed = 0.3, correctiveStrafeHeadingMultiplier = 0.15;
    private State state;
    private DcMotorEx fr, fl, br, bl;
    private DrivePath correctiveDrive;
    public Drivetrain(Hardware hardware, Telemetry telemetry, OpmodeType opmodeType) {
        super(hardware, telemetry);
        this.state = opmodeType == OpmodeType.TELE ? State.TELE_DRIVE : State.AUTONOMOUS;
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
                if (keybinds.check(Keybinds.D1Trigger.AUTO_AIM)) {
                    setState(State.CORRECT_TO_SHOOTING_POSITION);
                    break;
                }

                double[] powers = calculateDrivePowersWithGamepadInput();
                setDrivePowers(powers[0], powers[1], powers[2]);
                break;
            case CORRECT_TO_SHOOTING_POSITION:
                if (correctiveDrive != null && correctiveDrive.isFinished() || (g1.getLeftStickX() != 0 || g1.getLeftStickY() != 0 || g1.getRightStickX() != 0)) {
                    correctiveDrive.cancel();
                    setState(State.TELE_DRIVE);
                    break;
                }
                break;
            case AUTONOMOUS:
                // do nothing
                break;
        }
    }

    public double[] calculateDrivePowersWithGamepadInput() {
        double xSign = Math.signum(g1.getLeftStickX());
        double ySign = Math.signum(g1.getLeftStickY());
        double headingSign = Math.signum(g1.getRightStickX());

        double lateral = xSign * Math.abs(Math.pow(g1.getLeftStickX(), lateralScaling));
        double axial = ySign * -Math.abs(Math.pow(g1.getLeftStickY(), axialScaling));
        double heading = headingSign * -Math.max(minTurnSpeed, Math.abs(Math.pow(g1.getRightStickX(), headingScaling)));
        double correctiveStrafeHeading = lateral * correctiveStrafeHeadingMultiplier;
        heading = Range.clip(heading + correctiveStrafeHeading, -1, 1);

        return new double[]{ lateral, axial, heading };
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
    public void setState(State state) {
        this.state = state;
        if (state == State.TELE_DRIVE) {
            double[] powers = calculateDrivePowersWithGamepadInput();
            setDrivePowers(powers[0], powers[1], powers[2]);
        }
        else if (state == State.CORRECT_TO_SHOOTING_POSITION) {
            // find desired position (closest point on circle with radius of desired shooting distance)
            double goalX = robot.alliance == Alliance.BLUE ? Field.blueGoalX : Field.redGoalX;
            double rx = robot.pinpoint.pose().position.x, ry = robot.pinpoint.pose().position.y;
            Vector2d goalToRobot = new Vector2d(rx - goalX, ry - Field.goalY);
            goalToRobot = goalToRobot.div(Math.hypot(goalToRobot.x, goalToRobot.y));
            Vector2d desiredPosition = goalToRobot.times(Shooter.correctiveDriveParams.desiredNearShootDistance);
            double desiredHeading = Math.atan2(Field.goalY - desiredPosition.y, goalX - desiredPosition.y);
            Pose2d desiredPose = new Pose2d(desiredPosition, desiredHeading);

            // create drive path
            Tolerance tolerance = new Tolerance(Shooter.correctiveDriveParams.distTol, Math.toRadians(Shooter.correctiveDriveParams.headingTol));
            PathParams pathParams = new PathParams(Shooter.correctiveDriveParams.drivePIDs);
            pathParams.minSpeed = Shooter.correctiveDriveParams.minSpeed;
            pathParams.maxSpeed = Shooter.correctiveDriveParams.maxSpeed;
            pathParams.maxHeadingSpeed = Shooter.correctiveDriveParams.maxHeadingSpeed;
            Waypoint waypoint = new Waypoint(desiredPose, tolerance, pathParams);
            correctiveDrive = new DrivePath(this, robot.pinpoint, waypoint, telemetry);
            correctiveDrive.schedule();
        }
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

    public void correctToPose(Pose2d pose) {

    }
}
