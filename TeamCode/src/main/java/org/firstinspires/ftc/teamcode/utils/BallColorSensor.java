package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Hardware;

@Config
public class BallColorSensor extends Sensor {
    public static int[][] greenRanges = {
            { 50, 230 }, // red min/max
            { 150, 255 }, // green min/max
            { 50, 230 }  // blue min/max
    };
    public static int[][] purpleRanges = {
            { 150, 255 }, // red min/max
            { 0, 230 }, // green min/max
            { 150, 250 }  // blue min/max
    };
    public final String sensorName;
    private ColorSensor sensor;
    private int[] oldRgb;
    private final int[] rgb;

    public BallColorSensor(Hardware hardware, Telemetry telemetry, String sensorName, boolean turnedOn) {
        super(hardware, telemetry, turnedOn);
        this.sensorName = sensorName;
        oldRgb = new int[3];
        rgb = new int[3];
    }

    @Override
    public void declareHardware() {
        sensor = hardware.hardwareMap.get(ColorSensor.class, sensorName);
    }

    @Override
    public void cacheReading() {
        oldRgb = rgb;
        rgb[0] = sensor.red();
        rgb[1] = sensor.green();
        rgb[2] = sensor.blue();
    }

    public boolean seesBallFromLatestCache() {
        return inRange(purpleRanges, rgb) || inRange(greenRanges, rgb);
    }
    public boolean firstTimeSeeingBallFromLatestCache() {
        return seesBallFromLatestCache() && !inRange(purpleRanges, oldRgb) && !inRange(greenRanges, oldRgb);
    }

    private boolean inRange(int[][] range, int[] color) {
        for (int i=0; i<range.length; i++) // loop through each color channel
            if (color[i] < range[i][0] || color[i] > range[i][1]) // less than min or greater than max
                return false;
        return true;
    }

    public int[] getRgb() {
        return rgb;
    }
}
