package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opModesCompetition.tele.Keybinds;
import org.firstinspires.ftc.teamcode.utils.math.MathUtils;
import org.firstinspires.ftc.teamcode.utils.stateManagement.Subsystem;

public class Park extends Subsystem {
    public static double stowPosition = 0.01, parkPosition = 0.99, shootPosition = 0.3;
    private ServoImplEx leftServo, rightServo;
    private boolean parkedForShoot, enteredParkMode, fullyUp;
    public Park(Hardware hardware, Telemetry telemetry) {
        super(hardware, telemetry);
        parkedForShoot = false;
        enteredParkMode = false;
        this.fullyUp = false;
    }

    @Override
    public void declareHardware() {
        leftServo = hardware.getLeftParkServo();
        rightServo = hardware.getRightParkServo();
        setServoPositions(stowPosition);
    }

    @Override
    public void updateState() {
//        if (keybinds.check(Keybinds.D2Trigger.TOGGLE_PARK_FOR_SHOOT) && !enteredParkMode) {
//            parkedForShoot = !parkedForShoot;
//            setServoPositions(parkedForShoot ? shootPosition : stowPosition);
//        }

        if (keybinds.check(Keybinds.D2Trigger.RAISE_PARK) && enteredParkMode) {
            fullyUp = !fullyUp;
            setServoPositions(fullyUp ? parkPosition : stowPosition);
        }

        if (!enteredParkMode && keybinds.check(Keybinds.D2Trigger.ENTER_PARK_MODE)) {
            robot.drivetrain.setState(Drivetrain.State.TELE_SLOW_DRIVE);
            robot.drivetrain.setParkSlowDriveScale();
            robot.shooter.setState(Shooter.State.OFF);
            enteredParkMode = true;
        }
    }

    @Override
    public void printInfo() {
        telemetry.addLine("===RAISE_PARK===");
        telemetry.addData("left servo pos", MathUtils.format3(leftServo.getPosition()));
        telemetry.addData("right servo pos", MathUtils.format3(rightServo.getPosition()));
    }
    public void setServoPositions(double position) {
        leftServo.setPosition(position);
        rightServo.setPosition(position);
    }

    public boolean isParkedForShoot() {
        return parkedForShoot;
    }
}
