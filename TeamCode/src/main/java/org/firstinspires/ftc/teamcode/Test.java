package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class Test extends OpMode {
    int counter = 0;
    @Override
    public void init() {
        telemetry.addLine("init");
        telemetry.update();
    }

    @Override
    public void loop() {
        counter++;
        telemetry.addData("looping", counter);
        telemetry.update();
    }
}
