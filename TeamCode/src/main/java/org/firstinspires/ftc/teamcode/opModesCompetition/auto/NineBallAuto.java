package org.firstinspires.ftc.teamcode.opModesCompetition.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.opModesCompetition.tele.EverythingTele;
import org.firstinspires.ftc.teamcode.opModesCompetition.tele.Keybinds;
import org.firstinspires.ftc.teamcode.robot.Flipper;
import org.firstinspires.ftc.teamcode.robot.Hardware;
import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.Shooter;
import org.firstinspires.ftc.teamcode.utils.commands.InstantCommand;
import org.firstinspires.ftc.teamcode.utils.commands.WaitUntilCommand;
import org.firstinspires.ftc.teamcode.utils.generalOpModes.Alliance;
import org.firstinspires.ftc.teamcode.utils.generalOpModes.GamepadTracker;
import org.firstinspires.ftc.teamcode.utils.generalOpModes.OpmodeType;
import org.firstinspires.ftc.teamcode.utils.pidDrive.PathParams;
import org.firstinspires.ftc.teamcode.utils.pidDrive.DrivePath;
import org.firstinspires.ftc.teamcode.utils.pidDrive.Tolerance;
import org.firstinspires.ftc.teamcode.utils.pidDrive.Waypoint;

@Config
public class NineBallAuto extends OpMode {
    public static double fieldRotation = 90;
    public static class DrivePoses {
        public double startX = 31.15, startY = 62, startA = 90;
        public double shootX = 27.5, shootY = 27.5, shootA = 44;
        // first 3 balls
        public double collectX1 = 28, collectY1 = 13, collectA1 = 0;
        public double collectDriveThruX = 52, collectDriveThruY = 13, collectDriveThruA = 0;
        // second 3 balls
        public double collectX2 = 28, collectY2 = -13, collectA2 = 0;
        public double collect2DriveThruX = 52, collect2DriveThruY = -13, collect2DriveThruA = 0;
    }
    public static class DriveTolerances {
        public double shootDistTol = 1.5, shootHeadingTol = 3;
        // first 3 balls
        public double collect1XTol = 3, collect1YTol = 1.5, collect1HeadingTol = 3;
        public double collectDriveThruDistTol = 2, collectDriveThruHeadingTol = 5;
        // second 3 balls
        public double collect2XTol = 3, collect2YTol = 1.5, collect2HeadingTol = 3;
        public double collect2DriveThruDistTol = 2, collect2DriveThruHeadingTol = 5;
    }
    public static class DriveParams {
        public double[] shoot1PIDs = { 0.07, 0, 0.01, 0.02, 0, 0 };
        public double shoot1MinSpeed = 0.55, shoot1LateralWeight = 1.3, shoot2LateralWeight = 1.1;
        public double[] collect1PIDs = { 0.09, 0, 0, 0.05, 0, 0 };
        public double collect1MinSpeed = 0.4;
        public double[] collectDriveThruPIDs = { 0.06, 0, 0, 0.015, 0, 0 };
        public double collectDriveThruMaxSpeed = 0.8, collectDriveThruMinSpeed = 0.4;
        public double[] collect2PIDs = { 0.09, 0, 0, 0.05, 0, 0 };
        public double[] collect2DriveThruPIDs = { 0.06, 0, 0, 0.015, 0, 0 };
    }
    public static DrivePoses drivePoses = new DrivePoses();
    public static DriveTolerances driveTols = new DriveTolerances();
    public static DriveParams driveParams = new DriveParams();
    private Robot robot;
    private boolean done = false;
    public final Alliance alliance;
    public NineBallAuto(Alliance alliance) {
        this.alliance = alliance;
    }
    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(20);
        CommandScheduler.getInstance().reset();
        robot = new Robot(new Hardware(hardwareMap), telemetry, OpmodeType.AUTO, alliance);
        robot.declareHardware();
        robot.setInputInfo(new Keybinds(new GamepadTracker(null), new GamepadTracker(null)));

        Pose2d startPose = new Pose2d(drivePoses.startX, drivePoses.startY, Math.toRadians(drivePoses.startA));
        robot.pinpoint.setInitialPose(startPose);
    }

    @Override
    public void start() {
        Pose2d shootPose = new Pose2d(drivePoses.shootX, drivePoses.shootY, Math.toRadians(drivePoses.shootA));
        Pose2d collect1Pose = new Pose2d(drivePoses.collectX1, drivePoses.collectY1, Math.toRadians(drivePoses.collectA1));
        Pose2d collect1DriveThruPose = new Pose2d(drivePoses.collectDriveThruX, drivePoses.collectDriveThruY, Math.toRadians(drivePoses.collectDriveThruA));
        Pose2d collect2Pose = new Pose2d(drivePoses.collectX2, drivePoses.collectY2, Math.toRadians(drivePoses.collectA2));
        Pose2d collect2DriveThruPose = new Pose2d(drivePoses.collect2DriveThruX, drivePoses.collect2DriveThruY, Math.toRadians(drivePoses.collect2DriveThruA));

        if (alliance == Alliance.BLUE) {
            shootPose = mirrorPose(shootPose);
            collect1Pose = mirrorPose(collect1Pose);
            collect1DriveThruPose = mirrorPose(collect1DriveThruPose);
            collect2Pose = mirrorPose(collect2Pose);
            collect2DriveThruPose = mirrorPose(collect2DriveThruPose);
        }

        Tolerance shootTol = new Tolerance(driveTols.shootDistTol, Math.toRadians(driveTols.shootHeadingTol));
        PathParams shoot1Params = new PathParams(driveParams.shoot1PIDs);
        shoot1Params.minSpeed = driveParams.shoot1MinSpeed;
        shoot1Params.lateralWeight = driveParams.shoot1LateralWeight;

        Tolerance collect1Tol = new Tolerance(driveTols.collect1XTol, driveTols.collect1YTol, Math.toRadians(driveTols.collect1HeadingTol));
        PathParams collect1Params = new PathParams(driveParams.collect1PIDs);
        collect1Params.minSpeed = driveParams.collect1MinSpeed;

        Tolerance collectDriveThruTol = new Tolerance(driveTols.collectDriveThruDistTol, Math.toRadians(driveTols.collectDriveThruHeadingTol));
        PathParams collectDriveThruParams = new PathParams(driveParams.collectDriveThruPIDs);
        collectDriveThruParams.maxSpeed = driveParams.collectDriveThruMaxSpeed;
        collectDriveThruParams.minSpeed = driveParams.collectDriveThruMinSpeed;
        collectDriveThruParams.customEndCondition = () -> robot.pinpoint.pose().position.x >= drivePoses.collectDriveThruX;

        PathParams shoot2Params = new PathParams(driveParams.shoot1PIDs);
        shoot2Params.minSpeed = driveParams.shoot1MinSpeed;
        shoot2Params.lateralWeight = driveParams.shoot2LateralWeight;

        Tolerance collect2Tol = new Tolerance(driveTols.collect2XTol, driveTols.collect2YTol, driveTols.collect2HeadingTol);
        PathParams collect2Params = new PathParams(driveParams.collect2PIDs);

        Tolerance collect2DriveThruTol = new Tolerance(driveTols.collect2DriveThruDistTol, driveTols.collect2DriveThruHeadingTol);
        PathParams collect2DriveThruParams = new PathParams(driveParams.collect2DriveThruPIDs);

        Waypoint shoot1Waypoint = new Waypoint(shootPose, shootTol, shoot1Params);
        Waypoint collect1Waypoint = new Waypoint(collect1Pose, collect1Tol, collect1Params);
        Waypoint collectDriveThruWaypoint = new Waypoint(collect1DriveThruPose, collectDriveThruTol, collectDriveThruParams);
        Waypoint shoot2Waypoint = new Waypoint(shootPose, shootTol, shoot2Params);
        Waypoint collect2Waypoint = new Waypoint(collect2Pose, collect2Tol, collect2Params);
        Waypoint collect2DriveThruWaypoint = new Waypoint(collect2DriveThruPose, collect2DriveThruTol, collect2DriveThruParams);

        robot.intake.setState(Intake.State.PASSIVE_INTAKE);
        robot.shooter.setState(Shooter.State.TRACK_SHOOTER_SPEED);
        new SequentialCommandGroup(
                // first 1st 3
                new DrivePath(robot.drivetrain, robot.pinpoint, shoot1Waypoint, telemetry),
                new ParallelCommandGroup(
                        new WaitUntilCommand(() -> robot.shooter.canShootThreeNear(), Shooter.shooterParams.maxSpeedUpTime),
                        robot.drivetrain.headingLockCommand()
                ),
                robot.shootBallCommand(true, true),

                // collect 2nd 3
                new DrivePath(robot.drivetrain, robot.pinpoint, collect1Waypoint, telemetry),
                new InstantCommand(() -> {
                    robot.intake.setState(Intake.State.ON);
                    robot.shooter.setState(Shooter.State.TRACK_SHOOTER_SPEED);
                }),
                new DrivePath(robot.drivetrain, robot.pinpoint, collectDriveThruWaypoint, telemetry),
                new InstantCommand(() -> robot.intake.setState(Intake.State.OFF)),

                // shoot 2nd 3
                new DrivePath(robot.drivetrain, robot.pinpoint, shoot2Waypoint, telemetry),
                new ParallelCommandGroup(
                        new WaitUntilCommand(() -> robot.shooter.canShootThreeNear(), Shooter.shooterParams.maxSpeedUpTime),
                        robot.drivetrain.headingLockCommand()
                ),
                robot.shootBallCommand(true, false),

                // collect 3rd 3
                new DrivePath(robot.drivetrain, robot.pinpoint, collect2Waypoint, telemetry),
                new InstantCommand(() -> {
                    robot.intake.setState(Intake.State.ON);
                    robot.shooter.setState(Shooter.State.TRACK_SHOOTER_SPEED);
                }),
                new DrivePath(robot.drivetrain, robot.pinpoint, collect2DriveThruWaypoint, telemetry),
                new InstantCommand(() -> robot.intake.setState(Intake.State.OFF)),

                // shoot 3rd 3
                new DrivePath(robot.drivetrain, robot.pinpoint, shoot2Waypoint, telemetry),
                new ParallelCommandGroup(
                        new WaitUntilCommand(() -> robot.shooter.canShootThreeNear(), Shooter.shooterParams.maxSpeedUpTime),
                        robot.drivetrain.headingLockCommand()
                ),
                robot.shootBallCommand(true, true),

                // end everything
                new InstantCommand(() -> {
                    done = true;
                    robot.shooter.setState(Shooter.State.OFF);
                    robot.intake.setState(Intake.State.OFF);
                    robot.flipper.setState(Flipper.State.CLOSED);
                })
        ).schedule();
    }

    @Override
    public void loop() {
         CommandScheduler.getInstance().run();
         if (!done) {
             robot.update();

             double x = robot.pinpoint.pose().position.x, y = robot.pinpoint.pose().position.y, heading = robot.pinpoint.pose().heading.toDouble();
             TelemetryPacket packet = new TelemetryPacket();
             Canvas fieldOverlay = packet.fieldOverlay();
             fieldOverlay.setRotation(Math.toRadians(fieldRotation)); // rotate 90deg clockwise
             fieldOverlay.strokeCircle(x, y, 5);
             fieldOverlay.strokeLine(x, y, x + 5 * Math.cos(heading), y + 5 * Math.sin(heading));
             FtcDashboard.getInstance().sendTelemetryPacket(packet);

             telemetry.update();
         }
    }

    private Pose2d mirrorPose(Pose2d pose) {
        return new Pose2d(-pose.position.x, pose.position.y, Math.PI - pose.heading.toDouble());
    }
    @Override
    public void stop() {
        Pose2d endPose = robot.pinpoint.pose();
        EverythingTele.startX = endPose.position.x;
        EverythingTele.startY = endPose.position.y;
        EverythingTele.startA = endPose.heading.toDouble();
    }
}
