package org.firstinspires.ftc.teamcode.opModesTesting;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.utils.GamepadTracker;

@TeleOp(name="Motor Test", group = "Testing")
@Config
public class MotorTester extends OpMode {
    public static String motorConfigName = "motor1";
    private GamepadTracker g1;
    private DcMotorEx motor1;
    private double motorPower;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(20);
        g1 = new GamepadTracker(gamepad1);

        motor1 = hardwareMap.get(DcMotorEx.class, motorConfigName);
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        if (g1.isDpadUpClicked())
            motorPower += 0.05;
        else if (g1.isDpadDownClicked())
            motorPower -= 0.05;
        else if (g1.isAClicked())
            motorPower = 0;

        motorPower = Math.max(Math.min(0.99, motorPower), -0.99);

        motor1.setPower(motorPower);
        g1.update();

        telemetry.addData("d1 dpad up", g1.isDpadUpPressed());
        telemetry.addData("d1 dpad down", g1.isDpadDownPressed());
        telemetry.addData("d1 a", g1.isAPressed());
        telemetry.addData("increase power", "dpad up");
        telemetry.addData("decrease power", "dpad down");
        telemetry.addData("reset power", "a");

        telemetry.addData("motor power", motorPower);
        telemetry.update();
    }
}