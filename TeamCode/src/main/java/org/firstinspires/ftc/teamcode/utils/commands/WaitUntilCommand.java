package org.firstinspires.ftc.teamcode.utils.commands;

import java.util.function.BooleanSupplier;

public class WaitUntilCommand extends SimpleCommand {
    private final BooleanSupplier endCondition;
    private final double maxTime;
    public WaitUntilCommand(BooleanSupplier endCondition, double maxTime) {
        this.endCondition = endCondition;
        this.maxTime = maxTime;
    }
    @Override
    public void run() {}

    @Override
    public boolean isDone() {
        return endCondition.getAsBoolean() || getTimeRunning() >= maxTime;
    }
}
