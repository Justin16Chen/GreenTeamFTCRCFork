package org.firstinspires.ftc.teamcode.opModesTesting;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opModesCompetition.tele.Keybinds;
import org.firstinspires.ftc.teamcode.robot.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.Hardware;
import org.firstinspires.ftc.teamcode.utils.generalOpModes.GamepadTracker;
import org.firstinspires.ftc.teamcode.utils.generalOpModes.OpmodeType;
import org.firstinspires.ftc.teamcode.utils.math.MathUtils;
import org.firstinspires.ftc.teamcode.utils.pinpoint.PinpointLocalizer;

@Config
@TeleOp(name="Super Basic Drive", group="Testing")
public class SuperBasicDriveTele extends OpMode {
    public static double fieldRotation = 90;
    public static double startX = 47, startY = 54.7, startA = 90;
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

        dt = new Drivetrain(hardware, telemetry, OpmodeType.TELE);
        dt.declareHardware();
        dt.setInputInfo(new Keybinds(g1, g2));

        pinpoint = new PinpointLocalizer(hardwareMap, new Pose2d(startX, startY, Math.toDegrees(startA)), telemetry);
    }

    @Override
    public void loop() {
        g1.update();
        g2.update();

        dt.updateState();
        pinpoint.update();

        double x = pinpoint.pose().position.x, y = pinpoint.pose().position.y, heading = pinpoint.pose().heading.toDouble();
        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();
        fieldOverlay.setRotation(Math.toRadians(fieldRotation)); // rotate 90deg clockwise
        fieldOverlay.strokeCircle(x, y, 5);
        fieldOverlay.strokeLine(x, y, x + 5 * Math.cos(heading), y + 5 * Math.sin(heading));
        FtcDashboard.getInstance().sendTelemetryPacket(packet);

        telemetry.addLine("DRIVETRAIN");
        telemetry.addData("strafe", g1.getLeftStickX());
        telemetry.addData("axial", -g1.getLeftStickY());
        telemetry.addData("heading", -g1.getRightStickX());
        dt.printMotorPowers();

        telemetry.addLine();
        telemetry.addLine("PINPOINT");
        telemetry.addData("position (in)", MathUtils.format2(pinpoint.pose().position.x) + ", " + MathUtils.format2(pinpoint.pose().position.y));
        telemetry.addData("heading (deg)", MathUtils.format2(Math.toDegrees(pinpoint.pose().heading.toDouble())));

        telemetry.update();
    }
}
