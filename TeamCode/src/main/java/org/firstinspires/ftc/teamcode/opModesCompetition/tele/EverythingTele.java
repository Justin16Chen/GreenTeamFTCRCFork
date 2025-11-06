package org.firstinspires.ftc.teamcode.opModesCompetition.tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Hardware;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.BallColorSensor;
import org.firstinspires.ftc.teamcode.utils.generalOpModes.GamepadTracker;
import org.firstinspires.ftc.teamcode.utils.generalOpModes.OpmodeType;
import org.firstinspires.ftc.teamcode.utils.generalOpModes.ParentOpMode;
import org.firstinspires.ftc.teamcode.utils.stateManagement.Subsystem;

@TeleOp(name="Everything Tele", group = "Competition")
@Config
public class EverythingTele extends ParentOpMode {
    public static double startX = 0, startY = 0, startA = 0;
    private Robot robot;

    @Override
    public void initiation() {
        CommandScheduler.getInstance().reset();

        robot = new Robot(hardware, telemetry, OpmodeType.TELE);
        robot.declareHardware();
        robot.setInputInfo(new Keybinds(g1, g2));
        robot.pinpoint.setInitialPose(new Pose2d(startX, startY, startA));

    }

    @Override
    public void updateLoop() {

        robot.update();
        CommandScheduler.getInstance().run();

//        telemetry.addData("alliance", robot.alliance);
//        telemetry.addLine("=====SUBSYSTEMS=====");
//        for (Subsystem subsystem : robot.subsystems) {
//            subsystem.printInfo();
//            telemetry.addLine();
//        }
//
//        telemetry.addLine("=====SENSORS=====");
//        robot.pinpoint.printInfo();
//        telemetry.addLine();
//        for (BallColorSensor colorSensor : robot.colorSensors)
//            colorSensor.printInfo();
//
//        telemetry.update();
    }
}
