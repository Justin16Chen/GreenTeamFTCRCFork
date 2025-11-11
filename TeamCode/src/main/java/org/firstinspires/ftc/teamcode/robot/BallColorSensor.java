package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.math.MathUtils;
import org.firstinspires.ftc.teamcode.utils.stateManagement.Sensor;

@Config
public class BallColorSensor extends Sensor {
    public static double[][] greenRanges = {
            { 0.04, 0.23 }, // red min/max
            { 0.39, 0.7 }, // green min/max
            { 0.12, 0.51 }, // blue min/max
            { 2000, 10000 } // brightness min/max
    };
    public static double[][] purpleRanges = {
            { 0.16, 0.38 }, // red min/max
            { 0.2, 0.37 }, // green min/max
            { 0.37, 0.61 }, // blue min/max
            { 2500, 10000 } // brightness min/max

    };
    public final String sensorName;
    private ColorSensor sensor;
    private final double[] oldRgbb;
    public final double[] rgbb; // PERCENTS of each color channel as a whole (should add up to 1) to minimize lighting discrepancies; the second b is for brightness

    public BallColorSensor(Hardware hardware, Telemetry telemetry, String sensorName, boolean turnedOn) {
        super(hardware, telemetry, turnedOn);
        this.sensorName = sensorName;
        oldRgbb = new double[4];
        rgbb = new double[4];
    }

    @Override
    public void declareHardware() {
        sensor = hardware.hardwareMap.get(ColorSensor.class, sensorName);
    }

    @Override
    public void cacheReading() {
        oldRgbb[0] = rgbb[0];
        oldRgbb[1] = rgbb[1];
        oldRgbb[2] = rgbb[2];
        oldRgbb[3] = rgbb[3];
        rgbb[0] = sensor.red();
        rgbb[1] = sensor.green();
        rgbb[2] = sensor.blue();
        rgbb[3] = rgbb[0] + rgbb[1] + rgbb[2];
        for (int i=0; i<3; i++)
            rgbb[i] /= rgbb[3];
    }

    public boolean seesBallFromLatestCache() {
        return inRange(purpleRanges, rgbb) || inRange(greenRanges, rgbb);
    }
    public boolean firstTimeSeeingBallFromLatestCache() {
        return seesBallFromLatestCache() && !inRange(purpleRanges, oldRgbb) && !inRange(greenRanges, oldRgbb);
    }

    private boolean inRange(double[][] range, double[] color) {
        for (int i=0; i<range.length; i++) // loop through each color channel
            if (color[i] < range[i][0] || color[i] > range[i][1]) // less than min or greater than max
                return false;
        return true;
    }

    public void printRGB() {
        telemetry.addData("RGB", MathUtils.format3(rgbb));
    }
    public void printOldRGB() {
        telemetry.addData("old RGB", MathUtils.format3(oldRgbb));

    }

    public void printInfo() {
        telemetry.addData("turned on", isTurnedOn());
        telemetry.addData("sees ball", seesBallFromLatestCache());
    }
}
