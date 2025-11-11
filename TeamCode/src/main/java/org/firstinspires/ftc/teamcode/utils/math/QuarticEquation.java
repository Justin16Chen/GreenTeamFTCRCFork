package org.firstinspires.ftc.teamcode.utils.math;

public class QuarticEquation {
    public final double a, b, c, d, e;
    public QuarticEquation(double a, double b, double c, double d, double e) {
        this.a = a;
        this.b = b;
        this.c = c;
        this.d = d;
        this.e = e;
    }
    public double calculate(double x) {
        return a * x * x * x * x + b * x * x * x + c * x * x + d * x + e;
    }
}
