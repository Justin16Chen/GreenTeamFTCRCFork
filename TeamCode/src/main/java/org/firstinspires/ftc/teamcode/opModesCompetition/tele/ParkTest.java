package org.firstinspires.ftc.teamcode.opModesCompetition.tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Hardware;
import org.firstinspires.ftc.teamcode.robot.Park;
import org.firstinspires.ftc.teamcode.utils.generalOpModes.GamepadTracker;

@TeleOp(name="Park Test", group = "Competition")
@Config
public class ParkTest extends OpMode {
    private GamepadTracker g1;
    private Park park;
    private boolean out;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(20);
        Hardware hardware = new Hardware(hardwareMap);
        g1 = new GamepadTracker(gamepad1);

        park = new Park(hardware, telemetry);
        park.declareHardware();
        park.setServoPositions(Park.stowPosition);
        out = false;
    }

    @Override
    public void loop() {
        g1.update();

        if (g1.isAClicked()) {
            out = !out;
            park.setServoPositions(out ? Park.stowPosition : Park.parkPosition);
        }
        telemetry.addData("a", "toggle park");
        telemetry.addData("a pressed", g1.isAPressed());
        park.printInfo();
        telemetry.update();
    }
}