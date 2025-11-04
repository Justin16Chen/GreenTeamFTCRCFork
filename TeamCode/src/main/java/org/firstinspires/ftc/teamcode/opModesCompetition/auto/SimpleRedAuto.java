package org.firstinspires.ftc.teamcode.opModesCompetition.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.opModesCompetition.tele.Keybinds;
import org.firstinspires.ftc.teamcode.robot.Hardware;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.utils.commands.InstantCommand;
import org.firstinspires.ftc.teamcode.utils.generalOpModes.GamepadTracker;
import org.firstinspires.ftc.teamcode.utils.generalOpModes.OpmodeType;
import org.firstinspires.ftc.teamcode.utils.pidDrive.PathParams;
import org.firstinspires.ftc.teamcode.utils.pidDrive.DrivePath;
import org.firstinspires.ftc.teamcode.utils.pidDrive.Tolerance;
import org.firstinspires.ftc.teamcode.utils.pidDrive.Waypoint;

@Autonomous(name="Simple Drive Auto")
@Config
public class SimpleRedAuto extends OpMode {
    public static double fieldRotation = 90;
    public static class DrivePoses {
        public double startX = 46, startY = 54.6, startA = 90;
        public double shootX = 20, shootY = 20, shootA = 45;
        public double collectX1 = 30, collectY1 = 12, collectA1 = 0;
        public double collectX2 = 50, collectY2 = 12, collectA2 = 0;
    }
    public static class DriveTolerances {
        public double shootDistTol = 1.5, shootHeadingTol = 3;
        public double collect1XTol = 3, collect1YTol = 1.5, collect1HeadingTol = 3;
        public double collect2DistTol = 1.5, collect2HeadingTol = 5;
    }
    public static class DriveParams {
        public double[] shoot1PIDs = { 0.06, 0, 0.01, 0.02, 0, 0 };
        public double shoot1MinSpeed = 0.4, shoot1LateralWeight = 1.8;
        public double[] collect1PIDs = { 0.09, 0, 0, 0.03, 0, 0 };
        public double collect1MinSpeed = 0.4;
        public double[] collect2PIDs = { 0.06, 0, 0, 0.03, 0, 0 };
        public double collect2MaxSpeed = 0.5;
    }
    public static DrivePoses drivePoses = new DrivePoses();
    public static DriveTolerances driveTols = new DriveTolerances();
    public static DriveParams driveParams = new DriveParams();
    private Robot robot;
    private boolean done = false;
    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(20);
        CommandScheduler.getInstance().reset();
        robot = new Robot(new Hardware(hardwareMap), telemetry, OpmodeType.AUTO);
        robot.declareHardware();
        robot.setInputInfo(new Keybinds(new GamepadTracker(null), new GamepadTracker(null)));

        Pose2d startPose = new Pose2d(drivePoses.startX, drivePoses.startY, Math.toRadians(drivePoses.startA));
        robot.pinpoint.setInitialPose(startPose);
    }

    @Override
    public void start() {
        Pose2d shootPose = new Pose2d(drivePoses.shootX, drivePoses.shootY, Math.toRadians(drivePoses.shootA));
        Tolerance shootTol = new Tolerance(driveTols.shootDistTol, Math.toRadians(driveTols.shootHeadingTol));
        PathParams shootParams = new PathParams(driveParams.shoot1PIDs);
        shootParams.minSpeed = driveParams.shoot1MinSpeed;
        shootParams.lateralWeight = driveParams.shoot1LateralWeight;

        Pose2d collect1Pose = new Pose2d(drivePoses.collectX1, drivePoses.collectY1, Math.toRadians(drivePoses.collectA1));
        Tolerance collect1Tol = new Tolerance(driveTols.collect1XTol, driveTols.collect1YTol, Math.toRadians(driveTols.collect1HeadingTol));
        PathParams collect1Params = new PathParams(driveParams.collect1PIDs);
        collect1Params.minSpeed = driveParams.collect1MinSpeed;

        Pose2d collect2Pose = new Pose2d(drivePoses.collectX2, drivePoses.collectY2, Math.toRadians(drivePoses.collectA2));
        Tolerance collect2Tol = new Tolerance(driveTols.collect2DistTol, Math.toRadians(driveTols.collect2HeadingTol));
        PathParams collect2Params = new PathParams(driveParams.collect2PIDs);
        collect2Params.maxSpeed = driveParams.collect2MaxSpeed;

        Waypoint shootWaypoint = new Waypoint(shootPose, shootTol, shootParams);
        Waypoint collect1Waypoint = new Waypoint(collect1Pose, collect1Tol, collect1Params);
        Waypoint collect2Waypoint = new Waypoint(collect2Pose, collect2Tol, collect2Params);

        new SequentialCommandGroup(
                new DrivePath(robot.drivetrain, robot.pinpoint, shootWaypoint, telemetry),
                robot.shootBallCommand(),
                new DrivePath(robot.drivetrain, robot.pinpoint, collect1Waypoint, telemetry),
                new DrivePath(robot.drivetrain, robot.pinpoint, collect2Waypoint, telemetry),
                new InstantCommand(() -> done = true)
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
}
