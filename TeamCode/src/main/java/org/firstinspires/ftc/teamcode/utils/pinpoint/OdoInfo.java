package org.firstinspires.ftc.teamcode.utils.pinpoint;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Pose2d;

public class OdoInfo {
    public double x, y, headingRad;
    public OdoInfo(double x, double y, double headingRad) {
        this.x = x;
        this.y = y;
        this.headingRad = headingRad;
    }
    public OdoInfo() {
        x = 0;
        y = 0;
        headingRad = 0;
    }
    @NonNull
    @Override
    public OdoInfo clone() {
        return new OdoInfo(x, y, headingRad);
    }
}
