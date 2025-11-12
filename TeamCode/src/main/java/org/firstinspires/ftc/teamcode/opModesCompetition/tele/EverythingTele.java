package org.firstinspires.ftc.teamcode.opModesCompetition.tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opModesTesting.PosePredictionErrorRecorder;
import org.firstinspires.ftc.teamcode.robot.Hardware;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.BallColorSensor;
import org.firstinspires.ftc.teamcode.utils.generalOpModes.Alliance;
import org.firstinspires.ftc.teamcode.utils.generalOpModes.GamepadTracker;
import org.firstinspires.ftc.teamcode.utils.generalOpModes.OpmodeType;
import org.firstinspires.ftc.teamcode.utils.generalOpModes.ParentOpMode;
import org.firstinspires.ftc.teamcode.utils.math.HeadingCorrect;
import org.firstinspires.ftc.teamcode.utils.misc.TelemetryHelper;
import org.firstinspires.ftc.teamcode.utils.pinpoint.OdoInfo;
import org.firstinspires.ftc.teamcode.utils.stateManagement.Subsystem;

@Config
public class EverythingTele extends ParentOpMode {

    public static double startX = 0, startY = 0, startA = 0;
    private Robot robot;
    public final Alliance alliance;

    private Pose2d lastFramePredictedNextPose;

    public EverythingTele(Alliance alliance) {
        this.alliance = alliance;
    }
    @Override
    public void initiation() {
        CommandScheduler.getInstance().reset();

        robot = new Robot(hardware, telemetry, OpmodeType.TELE, alliance, new Pose2d(startX, startY, startA));
        robot.declareHardware();
        robot.setInputInfo(new Keybinds(g1, g2));

        lastFramePredictedNextPose = new Pose2d(startX, startY, startA);
        PosePredictionErrorRecorder.clearData();
    }

    @Override
    public void updateLoop() {

        robot.update();
        CommandScheduler.getInstance().run();

        printRobotInfo();

        Pose2d predictedPose = robot.pinpoint.getNextPoseSimple();
        Pose2d actualPose = robot.pinpoint.pose();
        TelemetryHelper.sendRobotPose(actualPose, predictedPose);

        OdoInfo error = new OdoInfo(predictedPose.position.x - actualPose.position.x,
                predictedPose.position.y - actualPose.position.y,
                HeadingCorrect.correctHeadingErrorRad(predictedPose.heading.toDouble() - actualPose.heading.toDouble()));
        PosePredictionErrorRecorder.predictionErrorsSimple.add(error);

        lastFramePredictedNextPose = predictedPose;
        telemetry.update();
    }

    private void printRobotInfo() {
        telemetry.addData("alliance", robot.alliance);
        telemetry.addLine("=====SUBSYSTEMS=====");
        for (Subsystem subsystem : robot.subsystems) {
            subsystem.printInfo();
            telemetry.addLine();
        }

        telemetry.addLine("=====SENSORS=====");
        robot.pinpoint.printInfo();
        telemetry.addLine();
        for (BallColorSensor colorSensor : robot.colorSensors)
            colorSensor.printInfo();
    }
}
