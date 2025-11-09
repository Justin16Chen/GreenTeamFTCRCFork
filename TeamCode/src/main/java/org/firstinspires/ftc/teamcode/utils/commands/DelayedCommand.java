package org.firstinspires.ftc.teamcode.utils.commands;

import com.arcrobotics.ftclib.command.Command;

public class DelayedCommand extends SimpleCommand {
    private final Command command;
    private final double delay;

    public DelayedCommand(Command command, double delay) {
        this.command = command;
        this.delay = delay;
    }

    @Override
    public void run() {
    }

    @Override
    public boolean isDone() {
        return false;
    }
}
