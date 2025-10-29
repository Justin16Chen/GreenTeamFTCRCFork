package org.firstinspires.ftc.teamcode.opModesTesting;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.robot.Flipper;
import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.robot.Shooter;
import org.firstinspires.ftc.teamcode.utils.generalOpModes.ParentOpMode;
import org.firstinspires.ftc.teamcode.utils.misc.MathUtils;

@TeleOp(name="Shooter Test", group = "Testing")
@Config
public class ShooterTest extends ParentOpMode {
    public static double joystickChangeIncrement = 0.01;
    private DcMotorEx intakeMotor;
    private ServoImplEx flipperServo;
    private boolean flipperBlocking;
    private DcMotorEx shooterMotor;
    private ServoImplEx leftServo, rightServo;
    private double targetServoPosition;

    private double power;

    @Override
    public void initiation() {
        intakeMotor = hardware.getIntakeMotor();
        flipperServo = hardware.getFlipperServo();

        shooterMotor = hardware.getShooterMotor();
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

        if (g1.isRBClicked())
            intakeMotor.setPower(intakeMotor.getPower() == 0 ? Intake.collectPower : 0);
        if (g1.isYClicked())
            intakeMotor.setPower(Intake.feedShooterPower);
        if (g1.isLBClicked()) {
            flipperBlocking = !flipperBlocking;
            flipperServo.setPosition(flipperBlocking ? Flipper.closePosition : Flipper.openPosition);
            intakeMotor.setPower(0);
        }

        if (g1.isDpadUpPressed()) {
            targetServoPosition += 0.01;
            targetServoPosition = Range.clip(targetServoPosition, 0, 1);
            setServoPosition(MathUtils.lerp(Shooter.hoodDownPosition, Shooter.hoodUpPosition, targetServoPosition));
        }
        else if (g1.isDpadDownPressed()) {
            targetServoPosition -= 0.01;
            targetServoPosition = Range.clip(targetServoPosition, 0, 1);
            setServoPosition(MathUtils.lerp(Shooter.hoodDownPosition, Shooter.hoodUpPosition, targetServoPosition));
        }

        // joystick increments
        if (!g1.isDpadUpPressed() && !g1.isDpadDownPressed())
            power -= g1.getLeftStickY() * joystickChangeIncrement;

        // reset
        if (g1.isAClicked())
            power = 0;
        power = Range.clip(power, -0.99, 0.99);
        shooterMotor.setPower(power);

//        shooterMotor.setVelocity(power, AngleUnit.DEGREES);
        g1.update();

        telemetry.addLine("===CONTROLS===");
        telemetry.addData("adjust shooter motor power", "left joystick");
        telemetry.addData("stop shooter motor", "a");
        telemetry.addData("adjust hood position", "dpad up, dpad down");
        telemetry.addData("toggle flipper", "left bumper");
        telemetry.addData("intake full power", "right bumper");
        telemetry.addData("intake half power", "y");
        telemetry.addLine();
        telemetry.addLine("===HOOD SERVOS===");
        telemetry.addData("target servo position", targetServoPosition);
        telemetry.addData("left servo position", leftServo.getPosition());
        telemetry.addData("right servo position", rightServo.getPosition());
        telemetry.addData("move hood up", "d2 dpad up");
        telemetry.addData("move hood down", "d2 dpad down");
        telemetry.addLine();
        telemetry.addLine("===MOTOR===");
        telemetry.addData("desired power (deg/sec)", power);
        telemetry.addData("motor power", shooterMotor.getPower());
        telemetry.addData("motor deg/sec", MathUtils.format2(shooterMotor.getVelocity(AngleUnit.DEGREES)));
        telemetry.addLine();
        telemetry.update();
    }
}