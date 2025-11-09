package org.firstinspires.ftc.teamcode.opModesCompetition.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.opModesCompetition.tele.EverythingTele;
import org.firstinspires.ftc.teamcode.opModesCompetition.tele.Keybinds;
import org.firstinspires.ftc.teamcode.robot.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.Flipper;
import org.firstinspires.ftc.teamcode.robot.Hardware;
import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.Shooter;
import org.firstinspires.ftc.teamcode.utils.commands.InstantCommand;
import org.firstinspires.ftc.teamcode.utils.commands.SimpleCommand;
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
        public double startX = 33, startY = 62, startA = 90;
        public double shootX = 30, shootY = 27.5, shootA = 44;
        // first 3 balls
        public double collectX1 = 28, collectY1 = 15, collectA1 = 0;
        public double collectDriveThruX = 53.5, collectDriveThruY = 13, collectDriveThruA = 0;
        public double gate1X = 48, gateY = 0, gateA = 0, gate2X = 53;
        // second 3 balls
        public double collectX2 = 28, collectY2 = -10, collectA2 = 0;
        public double collect2DriveThruX = 58, collect2DriveThruY = -13, collect2DriveThruA = 0;
        public double collect2ShootX = 51, collect2ShootY = -13, collect2ShootA = 0;
        public double collectX3 = 28, collectY3 = -33, collectA3 = 0;
        public double collect3DriveThruX = 58, collect3DriveThruY = -36, collect3DriveThruA = 0;
    }
    public static class DriveTolerances {
        public double shootDistTol = 1.5, shootHeadingDegTol = 2;
        // first 3 balls
        public double collect1XTol = 3, collect1YTol = 1.5, collect1HeadingDegTol = 3;
        public double collectDriveThruDistTol = 2, collectDriveThruHeadingDegTol = 5;
        public double gate1DistTol = 1.5, gate1HeadingTol = 5, gate2DistTol = 1.5, gate2HeadingTol = 3;
        // second 3 balls
        public double collect2XTol = 3, collect2YTol = 1.5, collect2HeadingDegTol = 3;
        public double collect2DriveThruDistTol = 2, collect2DriveThruHeadingDegTol = 5;
        public double collect2ShootDistTol = 2, collect2ShootHeadingDegTol = 3;
        // third 3 balls
        public double collect3XTol = 3, collect3YTol = 1.5, collect3HeadingDegTol = 3;
        public double collect3DriveThruDistTol = 2, collect3DriveThruHeadingDegTol = 5;
    }
    public static class DriveParams {
        public double[] shootPIDs = { 0.07, 0, 0.01, 0.017, 0, 0 };
        public double shootMinSpeed = 0.55, shootPreloadLateralWeight = 1.3, shoot1LateralWeight = 1.1;
        public double[] collect1PIDs = { 0.09, 0, 0, 0.019, 0, 0 };
        public double collect1MinSpeed = 0.4;
        public double[] collectDriveThruPIDs = { 0.08, 0, 0, 0.015, 0, 0 };
        public double collectDriveThruMaxSpeed = 0.75, collectDriveThruMinSpeed = 0.5;
        public double[] gatePIDs = { 0.05, 0, 0, 0.016, 0, 0 };
        public double gateMaxSpeed = 0.8, gateSlowDown = 0.7, gateMinSpeed = 0.3;
        public double[] collect2PIDs = { 0.08, 0, 0, 0.017, 0, 0 };
        public double collect2MaxSpeed = 0.75;
        public double[] collect2DriveThruPIDs = { 0.08, 0, 0, 0.015, 0, 0 };
        public double[] collect2ShootPIDs = { 0.05, 0, 0, 0.016, 0, 0 };
        public double collect2ShootSlowDown = 0.5;
        public double[] collect3PIDs = { 0.075, 0, 0, 0.018, 0, 0 };
        public double collect3MaxSpeed = 0.78;
        public double[] collect3DriveThruPIDs = { 0.08, 0, 0, 0.015, 0, 0 };
    }
    public static class SubsystemParams {
        public double firstShootTargetSpeed = 360;
        public long extraCollectTimeMs = 800;
        public double gateWaitTime = 0.8;
        public double pushGatePower = 0.5;
    }
    public static DrivePoses drivePoses = new DrivePoses();
    public static DriveTolerances driveTols = new DriveTolerances();
    public static DriveParams driveParams = new DriveParams();
    public static SubsystemParams subsystemParams = new SubsystemParams();
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
        Pose2d startPose = new Pose2d(drivePoses.startX, drivePoses.startY, Math.toRadians(drivePoses.startA));
        robot = new Robot(new Hardware(hardwareMap), telemetry, OpmodeType.AUTO, alliance, startPose);
        robot.declareHardware();
        robot.setInputInfo(new Keybinds(new GamepadTracker(null), new GamepadTracker(null)));

    }

    @Override
    public void start() {
        Pose2d shootPose = new Pose2d(drivePoses.shootX, drivePoses.shootY, Math.toRadians(drivePoses.shootA));
        Pose2d collect1Pose = new Pose2d(drivePoses.collectX1, drivePoses.collectY1, Math.toRadians(drivePoses.collectA1));
        Pose2d collect1DriveThruPose = new Pose2d(drivePoses.collectDriveThruX, drivePoses.collectDriveThruY, Math.toRadians(drivePoses.collectDriveThruA));
        Pose2d gate1Pose = new Pose2d(drivePoses.gate1X, drivePoses.gateY, Math.toRadians(drivePoses.gateA));
        Pose2d gate2Pose = new Pose2d(drivePoses.gate2X, drivePoses.gateY, Math.toRadians(drivePoses.gateA));
        Pose2d collect2Pose = new Pose2d(drivePoses.collectX2, drivePoses.collectY2, Math.toRadians(drivePoses.collectA2));
        Pose2d collect2DriveThruPose = new Pose2d(drivePoses.collect2DriveThruX, drivePoses.collect2DriveThruY, Math.toRadians(drivePoses.collect2DriveThruA));
        Pose2d collect2ShootPose = new Pose2d(drivePoses.collect2ShootX, drivePoses.collect2ShootY, Math.toRadians(drivePoses.collect2ShootA));
        Pose2d collect3Pose = new Pose2d(drivePoses.collectX3, drivePoses.collectY3, Math.toRadians(drivePoses.collectA3));
        Pose2d collect3DriveThruPose = new Pose2d(drivePoses.collect3DriveThruX, drivePoses.collect3DriveThruY, Math.toRadians(drivePoses.collect3DriveThruA));

        if (alliance == Alliance.BLUE) {
            shootPose = mirrorPose(shootPose);
            collect1Pose = mirrorPose(collect1Pose);
            collect1DriveThruPose = mirrorPose(collect1DriveThruPose);
            gate1Pose = mirrorPose(gate1Pose);
            gate2Pose = mirrorPose(gate2Pose);
            collect2Pose = mirrorPose(collect2Pose);
            collect2DriveThruPose = mirrorPose(collect2DriveThruPose);
            collect2ShootPose = mirrorPose(collect2ShootPose);
            collect3Pose = mirrorPose(collect3Pose);
            collect3DriveThruPose = mirrorPose(collect3DriveThruPose);
        }

        Tolerance shootTol = new Tolerance(driveTols.shootDistTol, driveTols.shootHeadingDegTol);
        PathParams shootPreloadParams = new PathParams(driveParams.shootPIDs);
        shootPreloadParams.minSpeed = driveParams.shootMinSpeed;
        shootPreloadParams.lateralWeight = driveParams.shootPreloadLateralWeight;

        Tolerance collect1Tol = new Tolerance(driveTols.collect1XTol, driveTols.collect1YTol, driveTols.collect1HeadingDegTol);
        PathParams collect1Params = new PathParams(driveParams.collect1PIDs);
        collect1Params.minSpeed = driveParams.collect1MinSpeed;

        Tolerance collectDriveThruTol = new Tolerance(driveTols.collectDriveThruDistTol, driveTols.collectDriveThruHeadingDegTol);
        PathParams collectDriveThruParams = new PathParams(driveParams.collectDriveThruPIDs);
        collectDriveThruParams.maxSpeed = driveParams.collectDriveThruMaxSpeed;
        collectDriveThruParams.minSpeed = driveParams.collectDriveThruMinSpeed;
        collectDriveThruParams.customEndCondition = () -> robot.pinpoint.pose().position.x >= drivePoses.collectDriveThruX;

        Tolerance gate1Tol = new Tolerance(driveTols.gate1DistTol, driveTols.gate1HeadingTol);
        Tolerance gate2Tol = new Tolerance(driveTols.gate2DistTol, driveTols.gate2HeadingTol);
        PathParams gatePathParams = new PathParams(driveParams.gatePIDs);
        gatePathParams.maxSpeed = driveParams.gateMaxSpeed;

        PathParams shoot1Params = new PathParams(driveParams.shootPIDs);
        shoot1Params.minSpeed = driveParams.shootMinSpeed;
        shoot1Params.lateralWeight = driveParams.shoot1LateralWeight;

        Tolerance collect2Tol = new Tolerance(driveTols.collect2XTol, driveTols.collect2YTol, driveTols.collect2HeadingDegTol);
        PathParams collect2Params = new PathParams(driveParams.collect2PIDs);
        collect2Params.maxSpeed = driveParams.collect2MaxSpeed;

        Tolerance collect2DriveThruTol = new Tolerance(driveTols.collect2DriveThruDistTol, driveTols.collect2DriveThruHeadingDegTol);
        PathParams collect2DriveThruParams = new PathParams(driveParams.collect2DriveThruPIDs);
        collect2DriveThruParams.maxSpeed = driveParams.collectDriveThruMaxSpeed;
        collect2DriveThruParams.minSpeed = driveParams.collectDriveThruMinSpeed;
        collect2DriveThruParams.customEndCondition = () -> robot.pinpoint.pose().position.x >= drivePoses.collect2DriveThruX;

        Tolerance collect2ShootTol = new Tolerance(driveTols.collect2ShootDistTol, driveTols.collect2ShootHeadingDegTol);
        PathParams collect2ShootParams = new PathParams(driveParams.collect2ShootPIDs);
        collect2ShootParams.slowDownPercent = driveParams.collect2ShootSlowDown;

        Tolerance collect3Tol = new Tolerance(driveTols.collect3XTol, driveTols.collect3YTol, driveTols.collect3HeadingDegTol);
        PathParams collect3Params = new PathParams(driveParams.collect3PIDs);
        collect3Params.maxSpeed = driveParams.collect3MaxSpeed;

        Tolerance collect3DriveThruTol = new Tolerance(driveTols.collect3DriveThruDistTol, driveTols.collect3DriveThruHeadingDegTol);
        PathParams collect3DriveThruParams = new PathParams(driveParams.collect3DriveThruPIDs);
        collect3DriveThruParams.maxSpeed = driveParams.collectDriveThruMaxSpeed;
        collect3DriveThruParams.minSpeed = driveParams.collectDriveThruMinSpeed;
        collect3DriveThruParams.customEndCondition = () -> robot.pinpoint.pose().position.x >= drivePoses.collect3DriveThruX;

        Waypoint shootPreloadWaypoint = new Waypoint(shootPose, shootTol, shootPreloadParams);
        Waypoint collect1Waypoint = new Waypoint(collect1Pose, collect1Tol, collect1Params);
        Waypoint collectDriveThruWaypoint = new Waypoint(collect1DriveThruPose, collectDriveThruTol, collectDriveThruParams);
        Waypoint gate1Waypoint = new Waypoint(gate1Pose, gate1Tol, gatePathParams);
        Waypoint gate2Waypoint = new Waypoint(gate2Pose, gate2Tol, gatePathParams);
        Waypoint shoot1Waypoint = new Waypoint(shootPose, shootTol, shoot1Params);
        Waypoint collect2Waypoint = new Waypoint(collect2Pose, collect2Tol, collect2Params);
        Waypoint collect2DriveThruWaypoint = new Waypoint(collect2DriveThruPose, collect2DriveThruTol, collect2DriveThruParams);
        Waypoint collect2ShootWaypoint = new Waypoint(collect2ShootPose, collect2ShootTol, collect2ShootParams);
        Waypoint collect3Waypoint = new Waypoint(collect3Pose, collect3Tol, collect3Params);
        Waypoint collect3DriveThruWaypoint = new Waypoint(collect3DriveThruPose, collect3DriveThruTol, collect3DriveThruParams);

        DrivePath shootPreloadDrive = new DrivePath(robot.drivetrain, robot.pinpoint, shootPreloadWaypoint, telemetry);
        DrivePath collectDrive = new DrivePath(robot.drivetrain, robot.pinpoint, collect1Waypoint, telemetry);
        DrivePath collectDriveThru =  new DrivePath(robot.drivetrain, robot.pinpoint, collectDriveThruWaypoint, telemetry);
        DrivePath gateDrive = new DrivePath(robot.drivetrain, robot.pinpoint, gate2Waypoint, telemetry);
        gateDrive.addWaypoint(gate1Waypoint);
        DrivePath shoot1Drive = new DrivePath(robot.drivetrain, robot.pinpoint, shoot1Waypoint, telemetry);
        DrivePath collect2Drive = new DrivePath(robot.drivetrain, robot.pinpoint, collect2Waypoint, telemetry);
        DrivePath collect2DriveThru = new DrivePath(robot.drivetrain, robot.pinpoint, collect2DriveThruWaypoint, telemetry);
        DrivePath shoot2Drive = new DrivePath(robot.drivetrain, robot.pinpoint, shoot1Waypoint, telemetry);
        shoot2Drive.addWaypoint(collect2ShootWaypoint);
        DrivePath collect3Drive = new DrivePath(robot.drivetrain, robot.pinpoint, collect3Waypoint, telemetry);
        DrivePath collect3DriveThru = new DrivePath(robot.drivetrain, robot.pinpoint, collect3DriveThruWaypoint, telemetry);
        DrivePath shoot3Drive = new DrivePath(robot.drivetrain, robot.pinpoint, shoot1Waypoint, telemetry);

        robot.shooter.setTargetSpeed(subsystemParams.firstShootTargetSpeed);
        robot.shooter.setState(Shooter.State.TRACK_SHOOTER_SPEED);
        new SequentialCommandGroup(
                // preload
                shootPreloadDrive,
//                new WaitUntilCommand(() -> robot.shooter.canShootThreeNear(), Shooter.nearParams.maxSpeedUpTime),
//                robot.shootBallCommand(true, true),

                // collect 1st 3
                collectDrive,
                new InstantCommand(() -> {
                    robot.intake.setState(Intake.State.ON);
                    robot.shooter.setState(Shooter.State.TRACK_SHOOTER_SPEED);
                }),
                collectDriveThru,

                // shoot 1st 3
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new WaitCommand(subsystemParams.extraCollectTimeMs),
                                new InstantCommand(() -> robot.intake.setState(Intake.State.OFF))
                        ),
                        new SequentialCommandGroup(
                                gateDrive,
                                pushAgainstGateCommand()
                        )
                ),
                shoot1Drive,

//                new WaitUntilCommand(() -> robot.shooter.canShootThreeNear(), Shooter.nearParams.maxSpeedUpTime),
//                robot.shootBallCommand(true, false),

                // collect 2nd 3
                collect2Drive,
                new InstantCommand(() -> robot.intake.setState(Intake.State.ON)),
                collect2DriveThru,
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new WaitCommand(subsystemParams.extraCollectTimeMs),
                                new InstantCommand(() -> robot.intake.setState(Intake.State.OFF))
                        ),
                        shoot2Drive
                ),
//
//                // shoot 2nd 3
//                new WaitUntilCommand(() -> robot.shooter.canShootThreeNear(), Shooter.nearParams.maxSpeedUpTime),
//                robot.shootBallCommand(true, true),

                // collect 3rd 3
                collect3Drive,
                new InstantCommand(() -> robot.intake.setState(Intake.State.ON)),
                collect3DriveThru,
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new WaitCommand(subsystemParams.extraCollectTimeMs),
                                new InstantCommand(() -> robot.intake.setState(Intake.State.OFF))
                        ),
                        shoot3Drive
                ),

//                new WaitUntilCommand(() -> robot.shooter.canShootThreeNear(), Shooter.nearParams.maxSpeedUpTime),
//                robot.shootBallCommand(true, true),

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

             if (robot.drivetrain.getState() != Drivetrain.State.AUTONOMOUS)
                 throw new IllegalStateException("drivetrain state of " + robot.drivetrain.getState() + " is not autonomous");
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

    private Command pushAgainstGateCommand() {
        return new SimpleCommand() {
            @Override
            public void run() {
                robot.drivetrain.setDrivePowers(0, subsystemParams.pushGatePower, 0);
            }
            @Override
            public boolean isDone() {
                return time > subsystemParams.gateWaitTime;
            }
        };
    }
}
