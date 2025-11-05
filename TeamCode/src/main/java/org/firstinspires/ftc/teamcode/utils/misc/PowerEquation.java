package org.firstinspires.ftc.teamcode.utils.misc;

public class PowerEquation {
    public final double a, b;
    public PowerEquation(double a, double b) {
        this.a = a;
        this.b = b;
    }
    public double calculate(double x) {
        return a * Math.pow(x, b);
    }
}
