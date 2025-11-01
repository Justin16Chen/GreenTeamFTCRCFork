package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.misc.MathUtils;
import org.firstinspires.ftc.teamcode.utils.stateManagement.Subsystem;

public class Drivetrain extends Subsystem {
    private DcMotorEx fr, fl, br, bl;
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
        setDrivePowers(g1.getLeftStickX(), -g1.getLeftStickY(), -g1.getRightStickX());
    }

    @Override
    public void printInfo() {
        telemetry.addLine("===DRIVETRAIN===");
        printMotorPowers();
    }

    public void setDrivePowers(double lateralPower, double axialPower, double headingPower) {
        double addValue = Math.round((100 * (axialPower * Math.abs(axialPower) + lateralPower * Math.abs(lateralPower)))) / 100.;
        double subtractValue = Math.round((100 * (axialPower * Math.abs(axialPower) - lateralPower * Math.abs(lateralPower)))) / 100.;
        setMotorPowers((addValue - headingPower), (subtractValue + headingPower), (subtractValue - headingPower), (addValue + headingPower));
    }
    public void setMotorPowers(double frontLeftPower, double frontRightPower, double backLeftPower, double backRightPower) {
        fl.setPower(frontLeftPower);
        fr.setPower(frontRightPower);
        bl.setPower(backLeftPower);
        br.setPower(backRightPower);
    }

    public void printMotorPowers() {
        telemetry.addData("dt powers (FL FR BL BR)", MathUtils.format3(fl.getPower()) + ", " + MathUtils.format3(fr.getPower()) + ", " + MathUtils.format3(bl.getPower()) + ", " + MathUtils.format3(br.getPower()));
    }
}
