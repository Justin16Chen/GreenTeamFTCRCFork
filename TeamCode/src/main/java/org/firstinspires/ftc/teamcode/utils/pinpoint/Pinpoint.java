package org.firstinspires.ftc.teamcode.utils.pinpoint;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class Pinpoint {
    public final PinpointLocalizer odo;

    public Pinpoint(HardwareMap hwMap) {
        odo = hwMap.get(PinpointLocalizer.class,"pinpoint");
        odo.setOffsets(5.562, -0, DistanceUnit.INCH); //these are tuned for 3110-0002-0001 Product Insight #1
        odo.setEncoderResolution(PinpointLocalizer.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(PinpointLocalizer.EncoderDirection.FORWARD, PinpointLocalizer.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();
    }

    public void update() {
        odo.update();
    }

    public Pose2D getPose() {
        return odo.getPosition();
    }
    public double x() {
        return odo.getPosX(DistanceUnit.INCH);
    }
    public double y() {
        return odo.getPosY(DistanceUnit.INCH);
    }
    public double headingDeg() {
        return odo.getHeading(AngleUnit.DEGREES);
    }
    public double headingRad() {
        return odo.getHeading(AngleUnit.RADIANS);
    }
}
