package org.firstinspires.ftc.teamcode.utils.misc;

public class AngleJavaClass {
    private double TAU = Math.PI * 2;


    public double norm(double angle){
        double modifiedAngle = angle % TAU;

        modifiedAngle = (modifiedAngle + TAU) % TAU;

        return modifiedAngle;
    }

    public double normDelta(double angleDelta){
        double modifiedAngle = norm(angleDelta);

        if(modifiedAngle > Math.PI){
            modifiedAngle -= TAU;
        }

        return modifiedAngle;
    }
}
