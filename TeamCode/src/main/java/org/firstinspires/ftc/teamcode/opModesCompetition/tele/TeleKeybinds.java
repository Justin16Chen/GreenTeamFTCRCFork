package org.firstinspires.ftc.teamcode.opModesCompetition.tele;

import org.firstinspires.ftc.teamcode.utils.generalOpModes.GamepadTracker;
import org.firstinspires.ftc.teamcode.utils.generalOpModes.Keybinds;

import java.util.HashMap;
import java.util.function.Predicate;

public class TeleKeybinds extends Keybinds {
    public TeleKeybinds(GamepadTracker g1, GamepadTracker g2) {
        super(g1, g2);
    }

    @Override
    protected void enterKeybinds(HashMap<D1Trigger, Predicate<GamepadTracker>> gt1Triggers, HashMap<D2Trigger, Predicate<GamepadTracker>> gt2Triggers) {
        gt1Triggers.put(D1Trigger.TOGGLE_INTAKE, GamepadTracker::isRBClicked);
        gt1Triggers.put(D1Trigger.AUTO_AIM, GamepadTracker::isLBClicked);
        gt1Triggers.put(D1Trigger.PARK, GamepadTracker::isAClicked);
    }
}
