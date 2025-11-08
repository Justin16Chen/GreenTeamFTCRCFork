package org.firstinspires.ftc.teamcode.opModesTesting;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.generalOpModes.GamepadTracker;
import org.firstinspires.ftc.teamcode.utils.misc.MathUtils;

@Config
@TeleOp(name="Shooter Speed Recorder")
public class ShooterSpeedRecorder extends OpMode {
    public static double recordIntervalMs = 50;
    public static int numDataEntries = 4;
    public static int numShotsToRecord = 15; // if you shoot 3 consecutively, it only counts as 1 shot - so this is 45 balls if you collect and shoot 3 everytime
    public static int recordAmountForEachShot = 100;
    public static double[][][] data = new double[numShotsToRecord][recordAmountForEachShot][numDataEntries];

    private GamepadTracker g1;

    public static void resetData() {
        data = new double[numShotsToRecord][recordAmountForEachShot][numDataEntries];
        currentShot = 0;
    }

    private static int currentShot = 0;
    public static int getCurrentShot() {
        return currentShot;
    }
    public static void incrementCurrentShot() {
        currentShot++;
    }
    private int currentShownShot = 0;
    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(20);
        g1 = new GamepadTracker(gamepad1);
    }

    @Override
    public void loop() {
        g1.update();

        if (gamepad1.y)
            resetData();
        if (g1.isDpadUpClicked())
            currentShownShot++;
        if (g1.isDpadDownClicked())
            currentShownShot--;

        currentShownShot = Math.max(0, Math.min(currentShot - 1, currentShownShot));

        telemetry.addLine("===CONTROLS===");
        telemetry.addData("reset speeds", "Y");
        telemetry.addData("scroll through recorded shots", "dpad up/down");
        telemetry.addLine();
        telemetry.addLine("===HYPER PARAMS===");
        telemetry.addData("current shot index", currentShownShot);
        telemetry.addData("last recorded shot index", currentShot - 1);
        telemetry.addLine();
        telemetry.addLine("===DATA (time, speed, power, hood) ===");
        double[][] shot = data[currentShownShot];
        for (double[] datum : shot)
            telemetry.addLine("t: " + MathUtils.format2(datum[0]) + ", s: " + MathUtils.format2(datum[1]) + ", p: " + MathUtils.format3(datum[2]) + ", h: " + MathUtils.format3(datum[3]));

        telemetry.update();
    }
}
