package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.generalOpModes.Keybinds;
import org.firstinspires.ftc.teamcode.utils.stateManagement.Subsystem;

public class Park extends Subsystem {
    public static double stowPosition = 0.01, parkPosition = 0.99;
    private ServoImplEx leftServo, rightServo;
    public Park(Hardware hardware, Telemetry telemetry) {
        super(hardware, telemetry);
    }

    @Override
    public void declareHardware() {
        leftServo = hardware.getLeftParkServo();
        rightServo = hardware.getRightParkServo();
    }

    @Override
    public void updateState() {
        if (keybinds.check(Keybinds.D1Trigger.PARK)) {
            setServoPositions(parkPosition);

            if (robot != null)
                robot.shooter.setState(Shooter.State.OFF);
        }
    }

    @Override
    public void printInfo() {
        telemetry.addLine("===PARK===");
        telemetry.addData("left servo pos", leftServo.getPosition());
        telemetry.addData("right servo pos", rightServo.getPosition());
    }
    public void setServoPositions(double position) {
        leftServo.setPosition(position);
        rightServo.setPosition(position);
    }
}
