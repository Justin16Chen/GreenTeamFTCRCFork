package org.firstinspires.ftc.teamcode.opModesCompetition.tele;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.utils.misc.Field;

@TeleOp(name="Reset Position Tele", group="Competition")
public class ResetPositionTele extends OpMode {
    @Override
    public void init() {
        EverythingTele.startX = Robot.backToCenterLength + Field.tileTeethLength * 0.5;
        EverythingTele.startY = Robot.width * 0.5 + Field.tileTeethLength * 0.5;
        EverythingTele.startA = 0;
    }

    @Override
    public void loop() {
        telemetry.addData("start position", EverythingTele.startX + ", " + EverythingTele.startY + " " + Math.toDegrees(EverythingTele.startA));
        telemetry.update();
    }
}
