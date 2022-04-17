package com.kennedyrobotics.hardware.pneumatics;


import com.kennedyrobotics.hardware.pneumatics.ISolenoid;

public class GhostSolenoid implements ISolenoid {

    private boolean state;

    @Override
    public void set(boolean on) {
        state = on;
    }

    @Override
    public boolean get() {
        return state;
    }

    @Override
    public void toggle() {
        state = !state;
    }

    @Override
    public int getChannel() {
        return 0;
    }

    @Override
    public boolean isDisabled() {
        return false;
    }

    @Override
    public void setPulseDuration(double durationSeconds) {

    }

    @Override
    public void startPulse() {

    }
}
