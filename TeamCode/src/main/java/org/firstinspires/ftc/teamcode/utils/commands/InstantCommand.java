package org.firstinspires.ftc.teamcode.utils.commands;

import org.firstinspires.ftc.teamcode.robot.Flipper;

public class InstantCommand extends SimpleCommand {

    private final Runnable runnable;
    public InstantCommand(Runnable runnable) {
        this.runnable = runnable;
    }
    @Override
    public void init() {
        runnable.run();
    }
    @Override
    public void run() {

    }

    @Override
    public boolean isDone() {
        return true;
    }
}
