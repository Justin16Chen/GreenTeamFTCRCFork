package org.firstinspires.ftc.teamcode.utils.commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Collections;
import java.util.Set;

public abstract class SimpleCommand implements Command {
    private final ElapsedTime timer;
    public SimpleCommand () {
        timer = new ElapsedTime();
    }
    public double getTimeRunning() {
        return timer.time();
    }
    @Override
    public Set<Subsystem> getRequirements() {
        return Collections.emptySet();
    }

    @Override
    public void initialize() {
        timer.startTime();
        init();
    }

    @Override
    public void execute() {
        run();
    }
    @Override
    public boolean isFinished() {
        return isDone();
    }

    // user is only meant to override these functions
    public void init() {}
    public abstract void run();
    public abstract boolean isDone();

}
