package org.firstinspires.ftc.teamcode.utils.misc;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.robot.Robot;

@Config
public class Field {
    public static double tileTeethLength = 0.8; // length of teeth of field tiles (in inches)
    public static double goalY = 61, blueGoalX = -62, redGoalX = 62;
    public static double blueParkX = 24 + 0.4 + 9, redParkX = -24 - 0.4 - 9, parkY = -48 + 0.4 + 9, parkADeg = 90;
    public static double blueResetX = 72 - Robot.frontToCenterLength, redResetX = -72 + Robot.frontToCenterLength, resetY = -72 + Robot.width * 0.5, blueResetADeg = 0, redResetADeg = 180;
}
