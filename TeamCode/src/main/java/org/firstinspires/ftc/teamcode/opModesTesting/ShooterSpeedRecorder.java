package org.firstinspires.ftc.teamcode.opModesTesting;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.misc.MathUtils;

@Config
@TeleOp(name="Shooter Speed Recorder")
public class ShooterSpeedRecorder extends OpMode {
    public static double recordIntervalMs = 50;
    public static int numDataEntries = 4;
    public static int recordAmount = 100;
    public static double[][] data = new double[recordAmount][numDataEntries];

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        if (gamepad1.y) {
            data = new double[recordAmount][numDataEntries];
        }

        telemetry.addData("reset speeds", "Y");
        telemetry.addLine();
        for (int i=0; i<data.length; i++)
            telemetry.addLine("time: " + Math.round(data[i][0]) + ", speed: " + Math.round(data[i][1]) + ", power: " + MathUtils.format3(data[i][2]) + ", hood pos: " + MathUtils.format3(data[i][3]));

        telemetry.update();
    }
}
