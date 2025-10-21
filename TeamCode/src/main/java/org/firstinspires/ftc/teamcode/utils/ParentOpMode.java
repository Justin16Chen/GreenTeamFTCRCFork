package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.opModesCompetition.tele.Keybinds;
import org.firstinspires.ftc.teamcode.robot.Hardware;

// requires FTC dashboard to use
public abstract class ParentOpMode extends OpMode {
    protected Hardware hardware;
    protected GamepadTracker g1, g2;
    protected Keybinds keybinds;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(20);

        hardware = new Hardware(hardwareMap);

        g1 = new GamepadTracker(gamepad1);
        g2 = new GamepadTracker(gamepad2);
        keybinds = new Keybinds(g1, g2);

        initiation();
    }



    @Override
    public void loop() {
        g1.update();
        g2.update();

        updateLoop();
    }

    public abstract void initiation();
    public abstract void updateLoop();
}
