package org.firstinspires.ftc.teamcode.opModesTesting;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Hardware;
import org.firstinspires.ftc.teamcode.robot.Park;
import org.firstinspires.ftc.teamcode.utils.GamepadTracker;

@TeleOp(name="Park Test", group = "Utility")
@Config
public class ParkTest extends OpMode {
    private GamepadTracker g1;
    private Park park;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(20);
        Hardware hardware = new Hardware(hardwareMap);
        g1 = new GamepadTracker(gamepad1);

        park = new Park(hardware, telemetry);
        park.declareHardware();
        park.setServoPositions(Park.stowPosition);
    }

    @Override
    public void loop() {
        g1.update();

        if (g1.isAClicked())
            park.setServoPositions(park.getServoPositions() == Park.stowPosition ? Park.parkPosition : Park.stowPosition);

        telemetry.addData("a", "toggle park");
        telemetry.addData("a pressed", g1.isAPressed());
        telemetry.addData("current park servo position", park.getServoPositions());
        telemetry.update();
    }
}