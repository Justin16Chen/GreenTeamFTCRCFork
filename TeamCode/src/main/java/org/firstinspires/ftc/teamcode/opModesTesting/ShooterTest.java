package org.firstinspires.ftc.teamcode.opModesTesting;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.robot.Flipper;
import org.firstinspires.ftc.teamcode.robot.Shooter;
import org.firstinspires.ftc.teamcode.utils.generalOpModes.ParentOpMode;
import org.firstinspires.ftc.teamcode.utils.misc.MathUtils;

@TeleOp(name="Shooter Test", group = "Testing")
@Config
public class ShooterTest extends ParentOpMode {
    public static boolean powerLeft = true, powerRight = true;
    public static double joystickChangeIncrement = 0.01;
    private DcMotorEx intakeMotor;
    private ServoImplEx flipperServo;
    private boolean flipperBlocking;
    private DcMotorEx leftShooterMotor, rightShooterMotor;
    private ServoImplEx leftServo, rightServo;
    private double targetServoPosition;

    private double intakePower;
    private double shooterPower;

    @Override
    public void initiation() {
        intakeMotor = hardware.getIntakeMotor();
        flipperServo = hardware.getFlipperServo();

        leftShooterMotor = hardware.getLeftShooterMotor();
        rightShooterMotor = hardware.getRightShooterMotor();
        leftServo = hardware.getLeftHoodServo();
        rightServo = hardware.getRightHoodServo();
        targetServoPosition = 0.5;
        setServoPosition(targetServoPosition);

    }

    private void setServoPosition(double pos) {
        leftServo.setPosition(pos);
        rightServo.setPosition(pos);
    }

    @Override
    public void updateLoop() {

        intakePower -= g1.getRightStickY() * joystickChangeIncrement;
        intakePower = Range.clip(intakePower, -0.99, 0.99);
        intakeMotor.setPower(intakePower);

        if (g1.isLBClicked()) {
            flipperBlocking = !flipperBlocking;
            flipperServo.setPosition(flipperBlocking ? Flipper.closePosition : Flipper.openPosition);
            intakeMotor.setPower(0);
        }

        if (g1.isDpadUpPressed()) {
            targetServoPosition += 0.01;
            targetServoPosition = Range.clip(targetServoPosition, 0, 1);
            setServoPosition(MathUtils.lerp(Shooter.hoodParams.downPosition, Shooter.hoodParams.upPosition, targetServoPosition));
        }
        else if (g1.isDpadDownPressed()) {
            targetServoPosition -= 0.01;
            targetServoPosition = Range.clip(targetServoPosition, 0, 1);
            setServoPosition(MathUtils.lerp(Shooter.hoodParams.downPosition, Shooter.hoodParams.upPosition, targetServoPosition));
        }

        // joystick increments
        shooterPower -= g1.getLeftStickY() * joystickChangeIncrement;
        shooterPower = Math.max(shooterPower, 0); // don't allow negative power so we don't cook motor axle adapters again

        // reset
        if (g1.isAClicked())
            shooterPower = 0;
        shooterPower = Range.clip(shooterPower, -0.99, 0.99);
        setShooterMotorPower(shooterPower);

        g1.update();

        telemetry.addLine("===CONTROLS===");
        telemetry.addData("adjust shooter motor power", "left joystick");
        telemetry.addData("stop shooter motor", "a");
        telemetry.addData("adjust hood position", "dpad up, dpad down");
        telemetry.addData("toggle flipper", "left bumper");
        telemetry.addData("adjust intake motor power", "right joystick");
        telemetry.addLine();
        telemetry.addLine("===HOOD SERVOS===");
        telemetry.addData("target servo position", targetServoPosition);
        telemetry.addData("left servo position", leftServo.getPosition());
        telemetry.addData("right servo position", rightServo.getPosition());
        telemetry.addData("move hood up", "d2 dpad up");
        telemetry.addData("move hood down", "d2 dpad down");
        telemetry.addLine();
        telemetry.addLine("===SHOOTER MOTOR===");
        telemetry.addData("desired shooter power", shooterPower);
        telemetry.addData("actual shooter power (l, r)", MathUtils.format3(leftShooterMotor.getPower()) + ", " + MathUtils.format3(rightShooterMotor.getPower()));
        telemetry.addData("motor deg/sec", MathUtils.format3(leftShooterMotor.getVelocity(AngleUnit.DEGREES)));
        telemetry.addLine();
        telemetry.addLine("===INTAKE & TRANSFER===");
        telemetry.addData("desired intake motor power", MathUtils.format3(intakePower));
        telemetry.addData("actual intake motor power", MathUtils.format3(intakeMotor.getPower()));
        telemetry.addData("flipper position", flipperServo.getPosition());
        telemetry.update();
    }

    private void setShooterMotorPower(double power) {
        if (powerLeft)
            leftShooterMotor.setPower(power);
        if (powerRight)
            rightShooterMotor.setPower(power);
    }
}