package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.utils.misc.PIDFController;
import org.firstinspires.ftc.teamcode.utils.stateManagement.StateSubsystem;
import org.firstinspires.ftc.teamcode.utils.stateManagement.Transition;

@Config
public class Shooter extends StateSubsystem<Shooter.State> {
    public static double maxSpeedUpTime = 5;
    public static double kp = 0.01, ki = 0, kd = 0, kf = 0;
    public static double hoodDownPosition = 0.99, hoodUpPosition = 0.4;
    public enum State {
        OFF,
        TRACK_SPEED
    }
    private DcMotorEx motor;
    private ServoImplEx leftServo, rightServo;
    private final PIDFController speedPid; // input error between current speed and target speed, output desired power
    private double targetSpeed;
    private double targetX, targetY;
    public Shooter(Hardware hardware, Telemetry telemetry) {
        super(hardware, telemetry);
        speedPid = new PIDFController(kp, ki, kd, kf);
        targetSpeed = 0;

        setInitialState(State.OFF);
        setTransitionFunction(Transition.Type.FROM_ANY_TO, State.OFF, () -> motor.setPower(0));
    }

    @Override
    public void declareHardware() {
        motor = hardware.getShooterMotor();
        leftServo = hardware.getLeftHoodServo();
        rightServo = hardware.getRightHoodServo();
    }

    @Override
    public void updateState() {
        switch (getState()) {
            case OFF:
                break;
            case TRACK_SPEED:
//                double hoodAngle = getHoodAngle();
//                double distance = Math.sqrt(robot.drivetrain.);
//                targetSpeed = calculateTargetFlywheelSpeed()
                motor.setPower(speedPid.update(motor.getVelocity(AngleUnit.DEGREES)));
                break;
        }
    }

    public boolean isReadyToShoot() {
        return true;
    }

    @Override
    public void printInfo() {
        telemetry.addLine("===SHOOTER===");
        telemetry.addData("state", getState());
        telemetry.addData("motor power", motor.getPower());
//        telemetry.addData("left servo power", leftServo.getPower());
//        telemetry.addData("right servo power", rightServo.getPower());
//        telemetry.addData("left servo position", leftServo.)
    }
}
