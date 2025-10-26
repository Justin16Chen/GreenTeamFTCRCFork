package org.firstinspires.ftc.teamcode.utils.misc;

public class LineEquation {
    public final double m, b;
    public LineEquation(double m, double b) {
        this.m = m;
        this.b = b;
    }
    public double calculate(double x) {
        return m * x + b;
    }
}
