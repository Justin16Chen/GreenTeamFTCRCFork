package org.firstinspires.ftc.teamcode.opModesTesting;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.Hardware;
import org.firstinspires.ftc.teamcode.utils.generalOpModes.GamepadTracker;
import org.firstinspires.ftc.teamcode.opModesCompetition.tele.TeleKeybinds;
import org.firstinspires.ftc.teamcode.utils.misc.MathUtils;
import org.firstinspires.ftc.teamcode.utils.pinpoint.PinpointLocalizer;

@TeleOp(name="Super Basic Drive", group="Testing")
public class SuperBasicDriveTele extends OpMode {
    private GamepadTracker g1, g2;
    private Drivetrain dt;
    private PinpointLocalizer pinpoint;
    @Override
    public void init() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(20);

        Hardware hardware = new Hardware(hardwareMap);
        g1 = new GamepadTracker(gamepad1);
        g2 = new GamepadTracker(gamepad2);

        dt = new Drivetrain(hardware, telemetry);
        dt.declareHardware();
        dt.setInputInfo(new TeleKeybinds(g1, g2));

        pinpoint = new PinpointLocalizer(hardwareMap, new Pose2d(0, 0, 0), telemetry);

    }

    @Override
    public void loop() {
        g1.update();
        g2.update();

        dt.updateState();
        pinpoint.update();

        telemetry.addLine("DRIVETRAIN");
        telemetry.addData("strafe", g1.getLeftStickX());
        telemetry.addData("axial", -g1.getLeftStickY());
        telemetry.addData("heading", -g1.getRightStickX());
        dt.printMotorPowers();

        telemetry.addLine();
        telemetry.addLine("PINPOINT");
        telemetry.addData("position (in)", MathUtils.format2(pinpoint.pose().position.x) + ", " + MathUtils.format2(pinpoint.pose().position.y));
        telemetry.addData("heading (deg)", MathUtils.format2(pinpoint.pose().heading.toDouble()));

        telemetry.update();
    }
}
