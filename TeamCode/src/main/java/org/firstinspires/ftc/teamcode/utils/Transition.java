package org.firstinspires.ftc.teamcode.utils;

import androidx.annotation.NonNull;

import java.util.Objects;

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
        return Objects.hash(from, to);
    }

    @NonNull
    @Override
    public String toString() {
        return "from " + from + " to " + to;
    }
}