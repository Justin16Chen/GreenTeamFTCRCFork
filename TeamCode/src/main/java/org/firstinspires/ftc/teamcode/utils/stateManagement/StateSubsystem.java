package org.firstinspires.ftc.teamcode.utils.stateManagement;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Hardware;

import java.util.HashMap;


// meant to be used to simplify complex state management
// can assign callback functions to run when transitioning from one state to another
public abstract class StateSubsystem<E extends Enum<E>> extends Subsystem {

    protected final HashMap<Transition<E>, Runnable> transitionFunctions;
    private E state;
    private double stateStartTime;
    public StateSubsystem(Hardware hardware, Telemetry telemetry) {
        super(hardware, telemetry);
        transitionFunctions = new HashMap<>();

    }
    protected void setTransitionFunction(E from, E to, Runnable function) {
        transitionFunctions.put(new Transition<>(from, to), function);
    }
    protected void setTransitionFunction(Transition.Type type, E transition, Runnable function) {
        transitionFunctions.put(new Transition<>(type, transition), function);
    }

    public E getState() {
        return state;
    }
    public double getStateTime() {
        return System.currentTimeMillis() - stateStartTime;
    }
    protected void setInitialState(E state) {
        this.state = state;
    }
    public void setState(E newState) {
        if (this.state == newState)
            return;
        stateStartTime = System.currentTimeMillis();
        E oldState = state;
        state = newState;
        Transition<E> transition = new Transition<>(oldState, newState);
//        Runnable runnable = transitionFunctions.get(transition);
        transitionFunctions.getOrDefault(new Transition<>(oldState, newState), () -> {}).run();
    }
}