package org.firstinspires.ftc.teamcode.utils;

// helper class to hold information about transitioning from one state to another
// used in StateSubsystem
public class Transition<E extends Enum<E>> {
    public enum Type {
        FROM_ANY_TO,
        TO_ANY_FROM
    }
    public final E from, to;
    public Transition(E from, E to) {
        this.from = from;
        this.to = to;
    }
    public Transition(Type type, E transition) {
        if (type == Type.FROM_ANY_TO) {
            this.from = transition;
            this.to = null;
        }
        else {
            this.from = null;
            this.to = transition;
        }
    }

    @Override
    public boolean equals(Object other) {
        if (other.getClass() != Transition.class)
            return false;
        Transition<E> otherTransition = (Transition<E>) other;
        boolean toMatches = to == null || otherTransition.to == null || to == otherTransition.to;
        boolean fromMatches = from == null || otherTransition.from == null || from == otherTransition.from;
        return toMatches && fromMatches;
    }
}