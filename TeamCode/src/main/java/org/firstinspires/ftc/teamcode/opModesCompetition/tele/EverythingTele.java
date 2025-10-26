package org.firstinspires.ftc.teamcode.opModesCompetition.tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Hardware;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.BallColorSensor;
import org.firstinspires.ftc.teamcode.utils.generalOpModes.GamepadTracker;

@TeleOp(name="Everything Tele", group = "Competition")
public class EverythingTele extends OpMode {
    private GamepadTracker g1, g2;
    private Robot robot;

    @Override
    public void init() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(20);

        Hardware hardware = new Hardware(hardwareMap);
        g1 = new GamepadTracker(gamepad1);
        g2 = new GamepadTracker(gamepad2);

        robot = new Robot(hardware, telemetry);
        robot.declareHardware();
        robot.setInputInfo(g1, g2, new TeleKeybinds(g1, g2));
    }

    @Override
    public void loop() {
        g1.update();
        g2.update();

        robot.update();
        robot.intake.printInfo();

        for (BallColorSensor colorSensor : robot.colorSensors)
            colorSensor.printInfo();
        
        telemetry.update();
    }

    private void printControls() {
        telemetry.addData("lx", gamepad1.left_stick_x);
        telemetry.addData("ly", -gamepad1.left_stick_y);
        telemetry.addData("turn", -gamepad1.right_stick_x);
        telemetry.addData("toggle intake", "right bumper");
    }
}
