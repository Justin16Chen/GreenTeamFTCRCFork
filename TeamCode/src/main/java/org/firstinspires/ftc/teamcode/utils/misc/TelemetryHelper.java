package org.firstinspires.ftc.teamcode.utils.misc;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;

public class TelemetryHelper {
    public static double fieldRotation = 90, robotRadius;

    public static void sendRobotPose(Pose2d ...poses) {
        for (Pose2d pose : poses) {
            double x = pose.position.x, y = pose.position.y, heading = pose.heading.toDouble();
            TelemetryPacket packet = new TelemetryPacket();
            Canvas fieldOverlay = packet.fieldOverlay();
            fieldOverlay.setRotation(Math.toRadians(fieldRotation)); // rotate 90deg clockwise
            fieldOverlay.strokeCircle(x, y, robotRadius);
            fieldOverlay.strokeLine(x, y, x + robotRadius * Math.cos(heading), y + robotRadius * Math.sin(heading));
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }
}
