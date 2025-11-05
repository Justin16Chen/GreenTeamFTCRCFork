package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.Command;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.commands.SimpleCommand;
import org.firstinspires.ftc.teamcode.utils.generalOpModes.OpmodeType;
import org.firstinspires.ftc.teamcode.utils.misc.MathUtils;
import org.firstinspires.ftc.teamcode.utils.stateManagement.Subsystem;

@Config
public class Drivetrain extends Subsystem {
    public static enum State {
        AUTONOMOUS,
        TELE_DRIVE,
        CORRECT_TO_POSITION
    }
    public static double lateralScaling = 3, axialScaling = 3, headingScaling = 3;
    public static double minTurnSpeed = 0.3, correctiveStrafeHeadingMultiplier = 0.15;
    private State state;
    private DcMotorEx fr, fl, br, bl;
    public Drivetrain(Hardware hardware, Telemetry telemetry, OpmodeType opmodeType) {
        super(hardware, telemetry);
        this.state = opmodeType == OpmodeType.TELE ? State.TELE_DRIVE : State.AUTONOMOUS;
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
        switch (state) {
            case TELE_DRIVE:
                double xSign = Math.signum(g1.getLeftStickX());
                double ySign = Math.signum(g1.getLeftStickY());
                double headingSign = Math.signum(g1.getRightStickX());
                double lateral = xSign * Math.abs(Math.pow(g1.getLeftStickX(), lateralScaling));
                double axial = ySign * -Math.abs(Math.pow(g1.getLeftStickY(), axialScaling));
                double heading = headingSign * -Math.max(minTurnSpeed, Math.abs(Math.pow(g1.getRightStickX(), headingScaling)));
                setDrivePowers(lateral, axial, heading);
                break;
            case AUTONOMOUS:
                break;
            case CORRECT_TO_POSITION:
                break;
        }
    }

    @Override
    public void printInfo() {
        telemetry.addLine("===DRIVETRAIN===");
        telemetry.addData("state", state);
        printMotorPowers();
    }

    public State getState() {
        return state;
    }
    public void setState(State state) {
        this.state = state;
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

    public void correctToPose(Pose2d pose) {

    }
}
