package org.firstinspires.ftc.teamcode.opModesCompetition.tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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
public class EverythingTele extends ParentOpMode {
    private Robot robot;

    @Override
    public void initiation() {
        robot = new Robot(hardware, telemetry, OpmodeType.TELE);
        robot.declareHardware();
        robot.setInputInfo(new TeleKeybinds(g1, g2));
        CommandScheduler.getInstance().reset();
    }

    @Override
    public void updateLoop() {

        robot.update();
        CommandScheduler.getInstance().run();

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

        telemetry.update();
    }
}
