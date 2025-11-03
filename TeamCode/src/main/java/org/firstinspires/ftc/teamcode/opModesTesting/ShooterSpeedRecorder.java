package org.firstinspires.ftc.teamcode.opModesTesting;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp(name="Shooter Speed Recorder")
public class ShooterSpeedRecorder extends OpMode {
    public static double firstShotSpeed = -1, secondShotSpeed = -1, thirdShotSpeed = -1;

    @Override
    public void init() {

    }

    @Override
    public void loop() {
        if (gamepad1.y) {
            firstShotSpeed = -1;
            secondShotSpeed = -1;
            thirdShotSpeed = -1;
        }
        telemetry.addData("reset speeds", "Y");
        telemetry.addLine();
        telemetry.addData("first shot speed", firstShotSpeed);
        telemetry.addData("second shot speed", secondShotSpeed);
        telemetry.addData("third shot speed", thirdShotSpeed);
        telemetry.update();
    }
}
