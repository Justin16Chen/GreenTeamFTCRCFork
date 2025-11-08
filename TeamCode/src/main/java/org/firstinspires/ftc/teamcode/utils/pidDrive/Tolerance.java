package org.firstinspires.ftc.teamcode.utils.pidDrive;

public class Tolerance {
    public double xTol;
    public double yTol;
    public double headingDegTol;
    public Tolerance(double xTol, double yTol, double headingDegTol) {
        this.xTol = xTol;
        this.yTol = yTol;
        this.headingDegTol = headingDegTol;
    }
    public Tolerance(double distTol, double headingDegTol) {
        xTol = distTol;
        yTol = distTol;
        this.headingDegTol = headingDegTol;
    }
    public Tolerance(double distTol) {
        xTol = distTol;
        yTol = distTol;
        headingDegTol = 360;
    }
}
