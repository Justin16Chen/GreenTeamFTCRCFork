package org.firstinspires.ftc.teamcode.opModesCompetition.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.opModesCompetition.tele.TeleKeybinds;
import org.firstinspires.ftc.teamcode.robot.Hardware;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.utils.generalOpModes.GamepadTracker;
import org.firstinspires.ftc.teamcode.utils.generalOpModes.OpmodeType;
import org.firstinspires.ftc.teamcode.utils.generalOpModes.ParentOpMode;
import org.firstinspires.ftc.teamcode.utils.pidDrive.DriveParams;
import org.firstinspires.ftc.teamcode.utils.pidDrive.DrivePath;
import org.firstinspires.ftc.teamcode.utils.pidDrive.Tolerance;
import org.firstinspires.ftc.teamcode.utils.pidDrive.Waypoint;

@Autonomous(name="Simple Drive Auto")
@Config
public class SimpleAuto extends OpMode {
    public static class DrivePoses {
        public double startX = 0, startY = 0, startA = -45;
        public double shootX = 0, shootY = 0, shootA = -45;
    }
    public static DrivePoses drivePoses = new DrivePoses();
    public static Tolerance shootTolerance = new Tolerance(2, 0.5);
    public static DriveParams shootDriveParams = new DriveParams(0.1, 0, 0, 0.8, 0, 0);
    private Robot robot;
    @Override
    public void init() {
        CommandScheduler.getInstance().reset();
        robot = new Robot(new Hardware(hardwareMap), telemetry, OpmodeType.AUTO);
        robot.declareHardware();
        robot.setInputInfo(new TeleKeybinds(new GamepadTracker(null), new GamepadTracker(null)));
    }

    @Override
    public void start() {
        Waypoint shootWaypoint = new Waypoint(new Pose2d(drivePoses.startX, drivePoses.startY, Math.toRadians(drivePoses.startA)), shootTolerance, shootDriveParams);
        new SequentialCommandGroup(
                new DrivePath(robot.drivetrain, robot.pinpoint, shootWaypoint, telemetry),
                robot.shootBallCommand()

        ).schedule();
        /*
        new ParallelAction(
            robot.update(),
            new SequentialAction(
                driveToShootPosition,
                shootBall(),
                new ParallelAction(
                    driveForwardSlowly,
                    new SequentialAction(
                        shootBall(),
                        shootBall()
                    )
                )
            )
        ).run();
         */
    }

    @Override
    public void loop() {
         CommandScheduler.getInstance().run();
        robot.update();
        telemetry.update();
    }
}
