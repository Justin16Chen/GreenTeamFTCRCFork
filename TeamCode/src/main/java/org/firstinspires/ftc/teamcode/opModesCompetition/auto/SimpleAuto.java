package org.firstinspires.ftc.teamcode.opModesCompetition.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opModesCompetition.tele.TeleKeybinds;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.utils.generalOpModes.GamepadTracker;
import org.firstinspires.ftc.teamcode.utils.generalOpModes.ParentOpMode;
import org.firstinspires.ftc.teamcode.utils.pidDrive.DriveParams;
import org.firstinspires.ftc.teamcode.utils.pidDrive.DrivePath;
import org.firstinspires.ftc.teamcode.utils.pidDrive.Tolerance;
import org.firstinspires.ftc.teamcode.utils.pidDrive.Waypoint;

@Autonomous(name="Simple Drive Auto")
public class SimpleAuto extends ParentOpMode {
    public static Pose2d shootPose = new Pose2d(24, 0, 0);
    public static Tolerance shootTolerance = new Tolerance(2, 0.5);
    public static DriveParams shootDriveParams = new DriveParams(0.5, 0, 0, 0.8, 0, 0);
    private Robot robot;
    @Override
    public void initiation() {
        g1 = new GamepadTracker(null);
        g2 = new GamepadTracker(null);
        CommandScheduler.getInstance().reset();
        robot = new Robot(hardware, telemetry);
        robot.declareHardware();
        robot.setInputInfo(new TeleKeybinds(g1, g2));
    }

    @Override
    public void start() {
        Waypoint shootWaypoint = new Waypoint(shootPose, shootTolerance, shootDriveParams);
        new SequentialCommandGroup(
                new DrivePath(robot.drivetrain, robot.pinpoint, shootWaypoint),
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
    public void updateLoop() {
         CommandScheduler.getInstance().run();
        robot.update();
    }
}
