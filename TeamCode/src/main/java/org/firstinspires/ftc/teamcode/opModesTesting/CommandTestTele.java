package org.firstinspires.ftc.teamcode.opModesTesting;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.utils.generalOpModes.ParentOpMode;
import org.firstinspires.ftc.teamcode.utils.commands.SimpleCommand;


@TeleOp(name="Command Test", group="Testing")
public class CommandTestTele extends ParentOpMode {
    private DcMotorEx intakeMotor;
    @Override
    public void initiation() {
        CommandScheduler.getInstance().reset();
        intakeMotor = hardware.getIntakeMotor();
    }

    @Override
    public void updateLoop() {
        CommandScheduler.getInstance().run();

        if (g1.isAClicked())
            new SimpleCommand() {
                @Override
                public void run() {
                    intakeMotor.setPower(getTimeRunning() / 3.);
                }

                @Override
                public boolean isDone() {
                    return getTimeRunning() > 3;
                }
            }.schedule();
    }
}
