package org.firstinspires.ftc.teamcode.utils.stateManagement;

import androidx.annotation.NonNull;

// helper class to hold information about transitioning from one state to another
// used in StateSubsystem
public class Transition<E extends Enum<E>> {
    public enum Type {
        FROM_ANY_TO,
        TO_ANY_FROM
    }
    public final E from, to;
    public final Runnable function;
    public Transition(E from, E to, Runnable function) {
        this.from = from;
        this.to = to;
        this.function = function;
    }
    public Transition(Type type, E transition, Runnable function) {
        this.function = function;
        switch (type) {
            case TO_ANY_FROM:
                this.from = transition;
                this.to = null;
                break;
            case FROM_ANY_TO:
            default:
                this.from = null;
                this.to = transition;
                break;
        }
    }

    @Override
    public boolean equals(Object other) {
        if (other.getClass() != Transition.class)
            return false;
        Transition<?> otherTransition = (Transition<?>) other;
        boolean fromMatches = from == null || otherTransition.from == null || from == otherTransition.from;
        boolean toMatches = to == null || otherTransition.to == null || to == otherTransition.to;
        return toMatches && fromMatches;
    }

    @Override
    public int hashCode() {
        return 0; // I don't think there is a better way to do this. welp
    }

    @NonNull
    @Override
    public String toString() {
        return "from " + from + " to " + to;
    }
}