package org.firstinspires.ftc.teamcode.opModesCompetition.tele;

import org.firstinspires.ftc.teamcode.utils.GamepadTracker;

import java.util.HashMap;
import java.util.function.Predicate;

public class Keybinds {
    public enum D1Trigger {
        TOGGLE_INTAKE,
        AUTO_AIM,
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

        gt1Triggers.put(D1Trigger.TOGGLE_INTAKE, GamepadTracker::isRBClicked);
        gt1Triggers.put(D1Trigger.AUTO_AIM, GamepadTracker::isLBClicked);
        gt1Triggers.put(D1Trigger.PARK, GamepadTracker::isAClicked);

        if (D1Trigger.values().length > gt1Triggers.size())
            throw new IllegalStateException("forgot to instantiate D1 trigger. enum size of " + D1Trigger.values().length + " is greater than " + gt1Triggers.size());
        if (D2Trigger.values().length > gt2Triggers.size())
            throw new IllegalStateException("forgot to instantiate D2 trigger. enum size of " + D2Trigger.values().length + " is greater than " + gt2Triggers.size());

    }

    public boolean check(D1Trigger trigger) {
        if (gt1Triggers.get(trigger) == null)
            throw new IllegalStateException("d1 trigger " + trigger + " is not defined in Keybinds");
        return gt1Triggers.get(trigger).test(g1);
    }

    public boolean check(D2Trigger trigger) {
        if (gt2Triggers.get(trigger) == null)
            throw new IllegalStateException("d2 trigger " + trigger + " is not defined in Keybinds");
        return gt2Triggers.get(trigger).test(g2);
    }
}