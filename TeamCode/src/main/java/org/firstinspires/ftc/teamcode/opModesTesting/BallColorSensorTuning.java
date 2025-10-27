package org.firstinspires.ftc.teamcode.opModesTesting;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.robot.BallColorSensor;
import org.firstinspires.ftc.teamcode.utils.generalOpModes.ParentOpMode;
import org.firstinspires.ftc.teamcode.utils.misc.MathUtils;

@TeleOp(name="Ball Color Sensor Tuning", group="Utility")
@Config
public class BallColorSensorTuning extends ParentOpMode {
    public static String colorSensorName = "backColorSensor";
    private Intake intake;
    private BallColorSensor colSensor;
    private double[] mins, maxes;
    private boolean tracking, seenBallWhileTracking;

    @Override
    public void initiation() {
        intake = new Intake(hardware, telemetry);
        intake.declareHardware();
        intake.setInputInfo(keybinds);

        colSensor = new BallColorSensor(hardware, telemetry, colorSensorName, true);
        colSensor.declareHardware();

        mins = new double[3];
        maxes = new double[3];
        tracking = false;
        seenBallWhileTracking = false;
    }

    @Override
    public void updateLoop() {

        if (g1.isAClicked()) {
            tracking = !tracking;
            if (tracking) {
                mins[0] = 1;
                mins[1] = 1;
                mins[2] = 1;
                maxes[0] = 0;
                maxes[1] = 0;
                maxes[2] = 0;
                seenBallWhileTracking = false;
            }
        }
        colSensor.update();

        if (tracking) {
            for (int i = 0; i < colSensor.rgbb.length; i++) {
                mins[i] = Math.min(mins[i], colSensor.rgbb[i]);
                maxes[i] = Math.max(maxes[i], colSensor.rgbb[i]);
            }

            if (colSensor.firstTimeSeeingBallFromLatestCache())
                seenBallWhileTracking = true;
        }

        telemetry.addLine("===CONTROLS===");
        telemetry.addData("toggle min/max tracking", "A");

        telemetry.addLine();

        telemetry.addLine("===INFO===");
        telemetry.addData("sees ball", colSensor.seesBallFromLatestCache());
        telemetry.addData("seen a ball while tracking", seenBallWhileTracking);
        telemetry.addLine();
        colSensor.printRGB();
        colSensor.printOldRGB();
        telemetry.addLine();
        telemetry.addData("currently tracking mins and maxes", tracking);
        telemetry.addData("mins", MathUtils.format2(mins));
        telemetry.addData("maxes", MathUtils.format2(maxes));

        telemetry.update();
    }
}
