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

//        printRobotInfo();

        telemetry.addData("velocity list size", robot.pinpoint.previousVelocities.size());
        telemetry.addData("acceleration list size", robot.pinpoint.previousAccelerations.size());

        Pose2d actualPose = robot.pinpoint.pose();
        TelemetryHelper.sendRobotPose(actualPose, lastFramePredictedNextPose);

        OdoInfo error = new OdoInfo(lastFramePredictedNextPose.position.x - actualPose.position.x,
                lastFramePredictedNextPose.position.y - actualPose.position.y,
                HeadingCorrect.correctHeadingErrorRad(lastFramePredictedNextPose.heading.toDouble() - actualPose.heading.toDouble()));
        PosePredictionErrorRecorder.predictionErrorsSimple.add(error);
        if (!robot.pinpoint.previousAccelerations.isEmpty())
            PosePredictionErrorRecorder.acceleration.add(robot.pinpoint.previousAccelerations.get(0));
        if (!robot.pinpoint.previousVelocities.isEmpty())
            PosePredictionErrorRecorder.velocity.add(robot.pinpoint.previousVelocities.get(0));

        Pose2d lastPose = robot.pinpoint.lastPose;
        OdoInfo controlGroupError = new OdoInfo(lastPose.position.x - actualPose.position.x,
                lastPose.position.y - actualPose.position.y,
                lastPose.heading.toDouble() - actualPose.heading.toDouble()
        );
        PosePredictionErrorRecorder.controlGroupError.add(controlGroupError);

        lastFramePredictedNextPose = robot.pinpoint.getNextPoseSimple();


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
