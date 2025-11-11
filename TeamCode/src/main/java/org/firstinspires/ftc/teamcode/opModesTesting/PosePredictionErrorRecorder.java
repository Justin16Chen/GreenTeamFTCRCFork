package org.firstinspires.ftc.teamcode.opModesTesting;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.math.MathUtils;

import java.util.ArrayList;
import java.util.Arrays;

@Config
@TeleOp(name="Pose Prediction Error Recorder", group="Data Recording")
public class PosePredictionErrorRecorder extends OpMode {
    public static int numDecimalPlacesToDisplay = 4;
    // double[] format = time, x, y, heading
    public static final ArrayList<double[]> predictionErrorsSimple = new ArrayList<>();
    public static final ArrayList<double[]> predictionErrorsAdvanced = new ArrayList<>();
    int mode = 0;
    @Override
    public void init() {
        mode = 0;
    }

    @Override
    public void loop() {
        if (gamepad1.a || gamepad2.a) {
            mode = (mode + 1) % 3;

            telemetry.addLine("===CONTROLS===");
            telemetry.addData("increment mode", "A");
            telemetry.addLine();
            telemetry.addData("available modes", "simple, advanced, comparison");
            telemetry.addLine();
            switch (mode) {
                case 0: telemetry.addLine("=== 0: SIMPLE ESTIMATION ERROR ==="); break;
                case 1: telemetry.addLine("=== 1: ADVANCED ESTIMATION ERROR ==="); break;
                case 2: telemetry.addLine("=== 2: COMPARISON ==="); break;
            }
            telemetry.addLine();

            if (mode != 2) {
                ArrayList<double[]> errorsToShow = mode == 0 ? predictionErrorsSimple : predictionErrorsAdvanced;
                for (int i = 0; i < errorsToShow.size(); i++) {
                    telemetry.addLine("t: " + Math.round(errorsToShow.get(i)[0]) +
                            "x: " + MathUtils.format(errorsToShow.get(i)[1], numDecimalPlacesToDisplay) +
                            "y: " + MathUtils.format(errorsToShow.get(i)[2], numDecimalPlacesToDisplay) +
                            "h: " + MathUtils.format(errorsToShow.get(i)[3], numDecimalPlacesToDisplay));
                }
            }
            else {
                double[] avgErrorSimple = calculateAverageError(predictionErrorsSimple);
                double[] avgErrorAdvanced = calculateAverageError(predictionErrorsAdvanced);
                double[] avgStdSimple = calculateAvgStandardDev(predictionErrorsSimple);
                double[] avgStdAdvanced = calculateAvgStandardDev(predictionErrorsAdvanced);

                telemetry.addLine("x error, y error, heading error");
                telemetry.addLine("average error simple method: " + Arrays.toString(avgErrorSimple));
                telemetry.addLine("average error advanced method: " + Arrays.toString(avgErrorAdvanced));
                telemetry.addLine("average standard dev simple method: " + Arrays.toString(avgStdSimple));
                telemetry.addLine("average standard dev advanced method: " + Arrays.toString(avgStdAdvanced));
            }

            telemetry.update();
        }
    }

    private double[] calculateAverageError(ArrayList<double[]> errors) {
        double[] avgErrors = new double[errors.get(0).length - 1]; // exclude first column b/c first column is time
        for (int i=1; i<errors.size(); i++) {
            for (int j = 0; j < errors.get(0).length; j++)
                avgErrors[j] += errors.get(i)[j];
        }
        for (int i=0; i<avgErrors.length; i++)
            avgErrors[i] /= errors.size();
        return avgErrors;
    }
    // variance = sum of the squared differences from the mean
    // standard dev = sqrt(variance)
    private double[] calculateAvgStandardDev(ArrayList<double[]> errors) {
        double[] avgErrors = calculateAverageError(errors);
        double[] stds = new double[avgErrors.length];

        for (int i=0; i<avgErrors.length; i++) {
            for (int j=1; j<errors.size(); j++) { // exclude first column b/c first column is time
                double diff = errors.get(j)[i] - avgErrors[i];
                stds[i] += diff * diff;
            }
        }
        for (int i=0; i<stds.length; i++)
            stds[i] = Math.sqrt(stds[i]);
        return stds;
    }
}
