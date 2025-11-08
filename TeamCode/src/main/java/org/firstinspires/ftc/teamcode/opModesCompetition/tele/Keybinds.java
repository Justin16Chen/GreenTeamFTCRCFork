package org.firstinspires.ftc.teamcode.opModesCompetition.tele;

import org.firstinspires.ftc.teamcode.utils.generalOpModes.GamepadTracker;

import java.util.HashMap;
import java.util.function.Predicate;

public class Keybinds {
    public enum D1Trigger {
        TOGGLE_INTAKE,
        MANUAL_OUTTAKE,
        SHOOT_NEAR,
        SHOOT_FAR,
        AUTO_AIM,
        PREPARE_FLYWHEEL,
        START_SHOOTING,
        CONTINUE_SHOOTING
    }
    public enum D2Trigger {
        PREPARE_FLYWHEEL,
        SET_PASSIVE_INTAKE,

        APPLY_PINPOINT_RESET_POWERS,
        RESET_PINPOINT_POSE,
        TOGGLE_PARK_MODE,
        RAISE_PARK
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
        gt1Triggers.put(D1Trigger.MANUAL_OUTTAKE, GamepadTracker::isLBPressed);
        gt1Triggers.put(D1Trigger.SHOOT_NEAR, GamepadTracker::isYClicked);
        gt1Triggers.put(D1Trigger.SHOOT_FAR, GamepadTracker::isBClicked);
        gt1Triggers.put(D1Trigger.AUTO_AIM, GamepadTracker::isLTPressed);
        gt1Triggers.put(D1Trigger.PREPARE_FLYWHEEL, GamepadTracker::isRBClicked);
        gt1Triggers.put(D1Trigger.START_SHOOTING, GamepadTracker::isRBClicked);
        gt1Triggers.put(D1Trigger.CONTINUE_SHOOTING, GamepadTracker::isRBPressed);

        gt2Triggers.put(D2Trigger.PREPARE_FLYWHEEL, GamepadTracker::isRBClicked);
        gt2Triggers.put(D2Trigger.SET_PASSIVE_INTAKE, GamepadTracker::isRBClicked);
        gt2Triggers.put(D2Trigger.APPLY_PINPOINT_RESET_POWERS, GamepadTracker::isLTPressed);
        gt2Triggers.put(D2Trigger.RESET_PINPOINT_POSE, GamepadTracker::isYClicked);
        gt2Triggers.put(D2Trigger.TOGGLE_PARK_MODE, GamepadTracker::isLTClicked);
        gt2Triggers.put(D2Trigger.RAISE_PARK, GamepadTracker::isRTClicked);

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