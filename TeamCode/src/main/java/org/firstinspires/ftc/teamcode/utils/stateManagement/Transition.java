package org.firstinspires.ftc.teamcode.utils.stateManagement;

import androidx.annotation.NonNull;

// helper class to hold information about transitioning from one state to another
// used in StateSubsystem
public class Transition<E extends Enum<E>> {
    public enum Type {
        FROM_ANY_TO,
        TO_ANY_FROM
    }
    private final Type type;
    public final E from, to;
    public Transition(E from, E to) {
        this.type = null;
        this.from = from;
        this.to = to;
    }
    public Transition(Type type, E transition) {
        this.type = type;
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
        Transition<E> otherTransition = (Transition<E>) other;
        boolean fromMatches = from == null || otherTransition.from == null || from == otherTransition.from;
        boolean toMatches = to == null || otherTransition.to == null || to == otherTransition.to;
//        throw new RuntimeException("this: " + this + " other: " + otherTransition + " | from: " + fromMatches + ", to: " + toMatches);
        return toMatches && fromMatches;
    }

    @Override
    public int hashCode() {
        int result = 1;
        if (from != null)
            result = 31 + from.hashCode();
        if (to != null)
            result = 31 * result + to.hashCode();
        return result;
//        return Objects.hash(type);
    }

    @NonNull
    @Override
    public String toString() {
        return "from " + from + " to " + to;
    }
}