package org.firstinspires.ftc.teamcode.opModesTesting;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.generalOpModes.GamepadTracker;
import org.firstinspires.ftc.teamcode.utils.generalOpModes.ParentOpMode;

@TeleOp(name="Controls test")
public class ControlsTest extends ParentOpMode {
    private GamepadTracker g1;
    private boolean clickedRB;
    @Override
    public void initiation() {
        g1 = new GamepadTracker(gamepad1);
    }

    @Override
    public void updateLoop() {
        if (g1.isRBClicked())
            clickedRB = true;

        telemetry.addData("clicked RB", clickedRB);
        telemetry.update();

    }
}
