package com.kennedyrobotics.hardware.pneumatics;

import com.kennedyrobotics.hardware.pneumatics.IDoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class GhostDoubleSolenoid implements IDoubleSolenoid {

    private DoubleSolenoid.Value value = DoubleSolenoid.Value.kOff;

    @Override
    public void set(DoubleSolenoid.Value value) {
        this.value = value;
    }

    @Override
    public DoubleSolenoid.Value get() {
        return value;
    }

    @Override
    public void toggle() {
        DoubleSolenoid.Value value = get();

        if (value == DoubleSolenoid.Value.kForward) {
            set(DoubleSolenoid.Value.kReverse);
        } else if (value == DoubleSolenoid.Value.kReverse) {
            set(DoubleSolenoid.Value.kForward);
        }
    }

    @Override
    public int getFwdChannel() {
        return 0;
    }

    @Override
    public int getRevChannel() {
        return 1;
    }

    @Override
    public boolean isFwdSolenoidDisabled() {
        return false;
    }

    @Override
    public boolean isRevSolenoidDisabled() {
        return false;
    }
}
