package org.firstinspires.ftc.teamcode.opModesCompetition.auto;

import org.firstinspires.ftc.teamcode.utils.generalOpModes.GamepadTracker;
import org.firstinspires.ftc.teamcode.utils.generalOpModes.Keybinds;

import java.util.HashMap;
import java.util.function.Predicate;

public class EmptyAutoKeybinds extends Keybinds {
    public EmptyAutoKeybinds() {
        super(null, null);
    }

    @Override
    protected void enterKeybinds(HashMap<D1Trigger, Predicate<GamepadTracker>> gt1Triggers, HashMap<D2Trigger, Predicate<GamepadTracker>> gt2Triggers) {

    }
}
