package org.firstinspires.ftc.teamcode.opModesCompetition.tele;

import org.firstinspires.ftc.teamcode.utils.generalOpModes.GamepadTracker;

import java.util.HashMap;
import java.util.function.Predicate;

public class Keybinds {
    public enum D1Trigger {
        TOGGLE_INTAKE,
        AUTO_AIM,
        PREPARE_FLYWHEEL,
        SHOOT,
        STOP_SHOOTING,
        PARK
    }
    public enum D2Trigger {
    }

    public final GamepadTracker g1, g2;
    private final HashMap<D1Trigger, Predicate<GamepadTracker>> gt1Triggers;
    private final HashMap<D2Trigger, Predicate<GamepadTracker>> gt2Triggers;


    public Keybinds(GamepadTracker g1, GamepadTracker g2) {
        this.g1 = g1;
        this.g2 = g2;
        gt1Triggers = new HashMap<>();
        gt2Triggers = new HashMap<>();

        gt1Triggers.put(D1Trigger.TOGGLE_INTAKE, GamepadTracker::isRTClicked);
        gt1Triggers.put(D1Trigger.AUTO_AIM, GamepadTracker::isLBClicked);
        gt1Triggers.put(D1Trigger.PREPARE_FLYWHEEL, GamepadTracker::isRBClicked);
        gt1Triggers.put(D1Trigger.SHOOT, GamepadTracker::isRBClicked);
        gt1Triggers.put(D1Trigger.STOP_SHOOTING, GamepadTracker::isLBClicked);
        gt1Triggers.put(D1Trigger.PARK, GamepadTracker::isYClicked);
    }


    public boolean check(D1Trigger trigger) {
        if (g1 == null)
            return false;
        if (gt1Triggers.get(trigger) == null)
            throw new IllegalStateException("d1 trigger " + trigger + " is not defined in Keybinds");
        return gt1Triggers.get(trigger).test(g1);
    }

    public boolean check(D2Trigger trigger) {
        if (g2 == null)
            return false;
        if (gt2Triggers.get(trigger) == null)
            throw new IllegalStateException("d2 trigger " + trigger + " is not defined in Keybinds");
        return gt2Triggers.get(trigger).test(g2);
    }
}