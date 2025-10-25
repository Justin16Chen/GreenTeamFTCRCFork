package org.firstinspires.ftc.teamcode.opModesCompetition.auto;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.utils.ParentOpMode;

public class SimpleAuto extends ParentOpMode {

    private Robot robot;
    @Override
    public void initiation() {
        robot = new Robot(hardware, telemetry);
        robot.setInputInfo(g1, g2,new EmptyAutoKeybinds());
    }

    @Override
    public void updateLoop() {
        robot.update();
    }
}
