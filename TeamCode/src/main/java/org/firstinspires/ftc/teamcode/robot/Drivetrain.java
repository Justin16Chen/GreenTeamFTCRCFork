package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.Subsystem;

public class Drivetrain extends Subsystem {
    DcMotorEx fr, fl, br, bl;
    public Drivetrain(Hardware hardware, Telemetry telemetry) {
        super(hardware, telemetry);
    }

    @Override
    public void declareHardware() {
        fr = hardware.getFRDriveMotor();
        fl = hardware.getFLDriveMotor();
        br = hardware.getBRDriveMotor();
        bl = hardware.getBLDriveMotor();
    }

    @Override
    public void updateState() {
        setDrivePowers(g1.getLeftStickX(), -g1.getLeftStickY(), g1.getRightStickX());
    }

    public void setDrivePowers(double lateralPower, double axialPower, double headingPower) {
        lateralPower *= -1; // because we define left as positive lateral (it is how it is defined in the field coordinate space)
        double addValue = Math.round((100 * (axialPower * Math.abs(axialPower) + lateralPower * Math.abs(lateralPower)))) / 100.;
        double subtractValue = Math.round((100 * (axialPower * Math.abs(axialPower) - lateralPower * Math.abs(lateralPower)))) / 100.;
        //double addValue = Math.round((100 * (axialPower + lateralPower))) / 100.;
        //double subtractValue = Math.round((100 * (axialPower - lateralPower))) / 100.;

        setMotorPowers((addValue - headingPower), (subtractValue + headingPower), (subtractValue - headingPower), (addValue + headingPower));
    }
    public void setMotorPowers(double frontLeftPower, double frontRightPower, double backLeftPower, double backRightPower) {
        fl.setPower(frontLeftPower);
        fr.setPower(frontRightPower);
        bl.setPower(backLeftPower);
        br.setPower(backRightPower);
    }
}
