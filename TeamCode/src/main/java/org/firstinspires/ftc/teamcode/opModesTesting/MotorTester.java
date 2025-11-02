package org.firstinspires.ftc.teamcode.opModesTesting;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.robot.Hardware;
import org.firstinspires.ftc.teamcode.utils.generalOpModes.GamepadTracker;

import java.util.Arrays;

@TeleOp(name="Motor Test", group = "Testing")
@Config
public class MotorTester extends OpMode {
    public static String[] motorConfigNames = { Hardware.intakeName, Hardware.shooterLeftName, Hardware.shooterRightName };
    public static double increment = 0.1;
    private GamepadTracker g1;
    private DcMotorEx motor1;
    private double motorPower;
    private int curMotorIndex;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(20);
        g1 = new GamepadTracker(gamepad1);

        motor1 = hardwareMap.get(DcMotorEx.class, motorConfigNames[curMotorIndex]);
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void init_loop() {
        g1.update();
        if (g1.isDpadUpClicked()) {
            curMotorIndex++;
            if (curMotorIndex >= motorConfigNames.length)
                curMotorIndex = motorConfigNames.length - 1;
            motor1 = hardwareMap.get(DcMotorEx.class, motorConfigNames[curMotorIndex]);
            motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        else if (g1.isDpadDownClicked()) {
            curMotorIndex--;
            if (curMotorIndex < 0)
                curMotorIndex = 0;
            motor1 = hardwareMap.get(DcMotorEx.class, motorConfigNames[curMotorIndex]);
            motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        telemetry.addData("all motor names", Arrays.toString(motorConfigNames));
        telemetry.addData("cur motor index", curMotorIndex);
        telemetry.addData("cur motor name", motorConfigNames[curMotorIndex]);
        telemetry.update();
    }

    @Override
    public void loop() {
        if (g1.isDpadUpClicked())
            motorPower += increment;
        else if (g1.isDpadDownClicked())
            motorPower -= increment;
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