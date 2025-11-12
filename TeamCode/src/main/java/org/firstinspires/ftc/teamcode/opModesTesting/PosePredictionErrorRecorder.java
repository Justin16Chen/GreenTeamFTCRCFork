package org.firstinspires.ftc.teamcode.opModesTesting;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.math.MathUtils;
import org.firstinspires.ftc.teamcode.utils.pinpoint.OdoInfo;

import java.util.ArrayList;
import java.util.Arrays;

@Config
@TeleOp(name="Pose Prediction Error Recorder", group="Data Recording")
public class PosePredictionErrorRecorder extends OpMode {
    public static int numDecimalPlacesToDisplay = 4;
    // double[] format = time, x, y, heading
    public static final ArrayList<OdoInfo> predictionErrorsSimple = new ArrayList<>();
    public static final ArrayList<OdoInfo> predictionErrorsAdvanced = new ArrayList<>();
    public static void clearData() {
        predictionErrorsSimple.clear();
        predictionErrorsAdvanced.clear();
    }
    int mode = 0;
    @Override
    public void init() {
        mode = 0;
    }

    @Override
    public void loop() {
        if (gamepad1.y || gamepad2.y)
            clearData();

        if (gamepad1.a || gamepad2.a) {
            mode = (mode + 1) % 3;

            telemetry.addLine("===CONTROLS===");
            telemetry.addData("increment mode", "A");
            telemetry.addData("clear data", "Y");
            telemetry.addLine();
            telemetry.addData("available modes", "simple, advanced, comparison");
            telemetry.addLine();
            switch (mode) {
                case 0: telemetry.addLine("=== 0: SIMPLE ESTIMATION ERROR ====="); break;
                case 1: telemetry.addLine("=== 1: ADVANCED ESTIMATION ERROR ==="); break;
                case 2: telemetry.addLine("=== 2: COMPARISON =================="); break;
            }
            telemetry.addLine();

            if (mode != 2) {
                ArrayList<OdoInfo> errorsToShow = mode == 0 ? predictionErrorsSimple : predictionErrorsAdvanced;
                for (int i = 0; i < errorsToShow.size(); i++) {
                    telemetry.addLine(
                            "x: " + MathUtils.format(errorsToShow.get(i).x, numDecimalPlacesToDisplay) +
                            "y: " + MathUtils.format(errorsToShow.get(i).y, numDecimalPlacesToDisplay) +
                            "h: " + MathUtils.format(errorsToShow.get(i).headingRad, numDecimalPlacesToDisplay));
                }
            }
            else {
                OdoInfo avgErrorSimple = calculateAverageError(predictionErrorsSimple);
                OdoInfo avgErrorAdvanced = calculateAverageError(predictionErrorsAdvanced);
                OdoInfo avgStdSimple = calculateAvgStandardDev(predictionErrorsSimple);
                OdoInfo avgStdAdvanced = calculateAvgStandardDev(predictionErrorsAdvanced);

                telemetry.addLine("x error, y error, heading error");
                telemetry.addLine("average error simple method: " + avgErrorSimple);
                telemetry.addLine("average error advanced method: " + avgErrorAdvanced);
                telemetry.addLine("average standard dev simple method: " + avgStdSimple);
                telemetry.addLine("average standard dev advanced method: " + avgStdAdvanced);
            }

            telemetry.update();
        }
    }

    private OdoInfo calculateAverageError(ArrayList<OdoInfo> errors) {
        OdoInfo avgErrors = new OdoInfo();
        for (int i=1; i<errors.size(); i++) {
            avgErrors.x += errors.get(i).x;
            avgErrors.y += errors.get(i).y;
            avgErrors.headingRad += errors.get(i).headingRad;
        }
        avgErrors.x /= errors.size();
        avgErrors.y /= errors.size();
        avgErrors.headingRad /= errors.size();
        return avgErrors;
    }
    // variance = sum of the squared differences from the mean
    // standard dev = sqrt(variance)
    private OdoInfo calculateAvgStandardDev(ArrayList<OdoInfo> errors) {
        OdoInfo avgErrors = calculateAverageError(errors);
        OdoInfo stds = new OdoInfo();

        for (int i=1; i<errors.size(); i++) { // exclude first column b/c first column is time
            double xDiff = errors.get(i).x - avgErrors.x;
            stds.x += xDiff * xDiff;

            double yDiff = errors.get(i).y - avgErrors.y;
            stds.y += yDiff * yDiff;

            double headingRadDiff = errors.get(i).headingRad - avgErrors.headingRad;
            stds.headingRad += headingRadDiff * headingRadDiff;
        }
        stds.x = Math.sqrt(stds.x);
        stds.y = Math.sqrt(stds.y);
        stds.headingRad = Math.sqrt(stds.headingRad);
        return stds;
    }
}
