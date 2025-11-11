package org.firstinspires.ftc.teamcode.utils.math;

public class QuadraticEquation {
    public final double a, b, c;
    public QuadraticEquation(double a, double b, double c) {
        this.a = a;
        this.b = b;
        this.c = c;
    }
    public double calculate(double x) {
        return a * x * x + b * x + c;
    }
}
