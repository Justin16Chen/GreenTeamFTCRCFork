package org.firstinspires.ftc.teamcode.utils.pidDrive;

public class Tolerance {
    public double xTol;
    public double yTol;
    public double headingRadTol;
    public Tolerance(double xTol, double yTol, double headingRadTol) {
        this.xTol = xTol;
        this.yTol = yTol;
        this.headingRadTol = headingRadTol;
    }
    public Tolerance(double distTol, double headingRadTol) {
        xTol = distTol;
        yTol = distTol;
        this.headingRadTol = headingRadTol;
    }
    public Tolerance(double distTol) {
        xTol = distTol;
        yTol = distTol;
        headingRadTol = 2 * Math.PI;
    }
}
