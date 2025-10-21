package org.firstinspires.ftc.teamcode.opModesTesting;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opModesCompetition.tele.Keybinds;
import org.firstinspires.ftc.teamcode.robot.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.Hardware;
import org.firstinspires.ftc.teamcode.utils.GamepadTracker;

@TeleOp(name="Super Basic Drive", group="Testing")
public class SuperBasicDriveTele extends OpMode {
    private GamepadTracker g1, g2;
    private Drivetrain dt;
    @Override
    public void init() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(20);

        Hardware hardware = new Hardware(hardwareMap);
        g1 = new GamepadTracker(gamepad1);
        g2 = new GamepadTracker(gamepad2);

        dt = new Drivetrain(hardware, telemetry);
        dt.declareHardware();
        dt.setInputInfo(new Keybinds(g1, g2));

    }

    @Override
    public void loop() {
        g1.update();
        g2.update();

        dt.updateState();

        telemetry.addData("strafe", g1.getLeftStickX());
        telemetry.addData("axial", -g1.getLeftStickY());
        telemetry.addData("heading", -g1.getRightStickX());
        dt.printMotorPowers();

        telemetry.update();
    }
}
