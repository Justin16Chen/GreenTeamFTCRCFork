package org.firstinspires.ftc.teamcode.opModesTesting;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.utils.generalOpModes.GamepadTracker;

@TeleOp(name="Shooter Test", group = "Testing")
@Config
public class ShooterTest extends OpMode {
    public static String motorConfigName = "shooter";
    public static int dpadChangeIncrement = 5, joystickChangeIncrement = 3;
    private GamepadTracker g1;
    private DcMotorEx shooter;
    private int desiredDPS; // degrees per second

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(20);
        g1 = new GamepadTracker(gamepad1);

        shooter = hardwareMap.get(DcMotorEx.class, motorConfigName);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        // dpad increments
        if (g1.isDpadUpClicked())
            desiredDPS += dpadChangeIncrement;
        else if (g1.isDpadDownClicked())
            desiredDPS -= dpadChangeIncrement;

        // joystick increments
        if (!g1.isDpadUpPressed() && !g1.isDpadDownPressed())
            desiredDPS -= (int) (g1.getLeftStickY() * joystickChangeIncrement);

        // reset
        if (g1.isAClicked())
            desiredDPS = 0;

        shooter.setVelocity(desiredDPS, AngleUnit.DEGREES);
        g1.update();

        telemetry.addData("d1 dpad up", g1.isDpadUpPressed());
        telemetry.addData("d1 dpad down", g1.isDpadDownPressed());
        telemetry.addData("d1 left stick y", g1.getLeftStickY());
        telemetry.addData("d1 a", g1.isAPressed());
        telemetry.addData("discretely increase power", "dpad up");
        telemetry.addData("discretely decrease power", "dpad down");
        telemetry.addData("continuously change power", "left stick y");
        telemetry.addData("reset power", "a");

        telemetry.addData("desired speed (deg/sec)", desiredDPS);
        telemetry.addData("motor power", shooter.getPower());
        telemetry.update();
    }
}