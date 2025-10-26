package org.firstinspires.ftc.teamcode.opModesCompetition.auto;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.utils.generalOpModes.ParentOpMode;

public class SimpleAuto extends ParentOpMode {

    private Robot robot;
    @Override
    public void initiation() {
        robot = new Robot(hardware, telemetry);
        robot.setInputInfo(g1, g2,new EmptyAutoKeybinds());
    }

    @Override
    public void start() {
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
        // CommandScheduler.update();
        robot.update();
    }
}
