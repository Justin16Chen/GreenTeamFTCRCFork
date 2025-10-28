package org.firstinspires.ftc.teamcode.opModesCompetition.auto;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.utils.generalOpModes.ParentOpMode;
import org.firstinspires.ftc.teamcode.utils.pidDrive.DriveParams;
import org.firstinspires.ftc.teamcode.utils.pidDrive.DrivePath;
import org.firstinspires.ftc.teamcode.utils.pidDrive.Tolerance;
import org.firstinspires.ftc.teamcode.utils.pidDrive.Waypoint;

public class SimpleAuto extends ParentOpMode {
    public static Pose2D shootPose = new Pose2D(DistanceUnit.INCH, 10, 10, AngleUnit.RADIANS, 0);
    public static Tolerance shootTolerance = new Tolerance(1, 0.5);
    public static DriveParams shootDriveParams = new DriveParams();
    private Robot robot;
    @Override
    public void initiation() {
        CommandScheduler.getInstance().reset();
        robot = new Robot(hardware, telemetry);
        robot.setInputInfo(g1, g2, new EmptyAutoKeybinds());

    }

    @Override
    public void start() {
        Waypoint shootWaypoint = new Waypoint(shootPose, shootTolerance, shootDriveParams);
        new SequentialCommandGroup(
                new DrivePath(robot.drivetrain, robot.odo, shootWaypoint),
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
