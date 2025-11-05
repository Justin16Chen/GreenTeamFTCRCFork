package org.firstinspires.ftc.teamcode.opModesTesting;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Hardware;
import org.firstinspires.ftc.teamcode.robot.Light;

import java.time.OffsetDateTime;

@TeleOp(name="light test", group="Testing")
public class LightTest extends OpMode {
    Light light;
    @Override
    public void init() {
        Hardware hardware = new Hardware(hardwareMap);
        light = new Light(hardware, telemetry);
        light.declareHardware();
    }

    @Override
    public void loop() {
        if (gamepad1.y) {
            light.setState(Light.State.FLASH);
            light.setFlashValues(Light.params.dim, Light.params.bright);
        }
        if (gamepad1.x) {
            light.setState(Light.State.FLASH);
            light.setFlashValues(Light.params.off, Light.params.dim);
        }
        else if (gamepad1.a) {
            light.setState(Light.State.PASSIVE);
            light.setPassiveValue(Light.params.dim);
        }
        else if (gamepad1.b) {
            light.setState(Light.State.PASSIVE);
            light.setPassiveValue(Light.params.off);
        }

        light.updateState();

        telemetry.addData("flash bright", "y");
        telemetry.addData("flash dim", "x");
        telemetry.addData("dim", "a");
        telemetry.addData("off", "b");

        telemetry.update();
    }
}
