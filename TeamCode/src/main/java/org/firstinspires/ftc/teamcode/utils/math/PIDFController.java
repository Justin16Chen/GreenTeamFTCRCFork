package org.firstinspires.ftc.teamcode.utils.math;
import com.qualcomm.robotcore.util.Range;

public class PIDFController {

    private double target;
    public double kP, kI, kD, kF;
    private double integral;
    private boolean shouldReset;

    private double previousTime, previousError;

    private double lowerInputBound = Double.NEGATIVE_INFINITY, higherInputBound = Double.POSITIVE_INFINITY;
    private double lowerOutputBound = Double.NEGATIVE_INFINITY, higherOutputBound = Double.POSITIVE_INFINITY;

    public PIDFController(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;

        shouldReset = true;
    }

    public void setPIDValues(double kP, double kI, double kD, double kF){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }

    public double getTarget() {
        return target;
    }

    public void setTarget(double target) {
        this.target = target;
    }

    public void setInputBounds(double lowerInputBound, double higherInputBound) {
        this.lowerInputBound = lowerInputBound;
        this.higherInputBound = higherInputBound;
    }

    public void setOutputBounds(double lowerOutputBound, double higherOutputBound) {
        this.lowerOutputBound = lowerOutputBound;
        this.higherOutputBound = higherOutputBound;
    }

    public double getLowerInputBound(){
        return this.lowerInputBound;
    }

    public double getUpperInputBound(){
        return this.higherInputBound;
    }


    public void reset() {
        shouldReset = true;
    }

    public double update(double value) {
        value = Range.clip(value, lowerInputBound, higherInputBound);

        double error = target - value;

        return updateWithError(error);
    }

    public double updateWithError(double error) {
        if (Double.isNaN(error) || Double.isInfinite(error))
            return 0;

        double proportional = kP * error;
        double derivative;
        double feedForward = kF;

        double currentTime = System.currentTimeMillis() / 1000.0;

        if (shouldReset) {
            shouldReset = false;
            integral = 0;
            derivative = 0;
            previousError = error;
        } else {
            double dT = currentTime - previousTime;
            integral += kI * error * dT;
            derivative = kD * (error - previousError) / dT;
        }

        previousTime = currentTime;
        previousError = error;

        double correction = proportional + integral + derivative + feedForward;

        return Math.signum(correction) * Range.clip(Math.abs(correction),
                lowerOutputBound, higherOutputBound);
    }
}
