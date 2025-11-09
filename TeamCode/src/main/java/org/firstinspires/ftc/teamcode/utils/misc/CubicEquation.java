package org.firstinspires.ftc.teamcode.utils.misc;

public class CubicEquation {
    public final double a, b, c, d;
    public CubicEquation(double a, double b, double c, double d) {
        this.a = a;
        this.b = b;
        this.c = c;
        this.d = d;
    }
    public double calculate(double x) {
        return a * x * x * x + b * x * x + c * x + d;
    }
}
