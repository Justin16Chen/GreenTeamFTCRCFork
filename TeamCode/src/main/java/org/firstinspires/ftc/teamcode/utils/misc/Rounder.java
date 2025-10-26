package org.firstinspires.ftc.teamcode.utils.misc;

import java.text.DecimalFormat;

// helps for concise, easy printing to telemetry
public class Rounder {
    public static DecimalFormat df = new DecimalFormat("#.##");

    public static String format2(Number num) {
        return format(num, 2);
    }
    public static String format(Number num, int decimalPlaces) {
        String decimals = "";
        for (int i=0; i<decimalPlaces; i++)
            decimals += "#";
        DecimalFormat customDf = new DecimalFormat("#." + decimals);
        return customDf.format(num);
    }
    public static String format2(double[] nums) {
        String total = "";
        for (double num : nums)
            total += format2(num) + ", ";
        return total.substring(0, total.length() - 2);
    }
}
