package com.kennedyrobotics.command;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * This is simply a wrapper that lets you name the instant commands on creation
 * Just a utility to make debugging buttons cleaner.
 *
 * Taken from: https://github.com/Team2470/FRC-2020-robot/blob/develop/src/main/java/bjorg/command/NamedInstantCommand.java
 */
public class NamedInstantCommand extends InstantCommand {

    private boolean m_runsWhenDisabled = false;

    public NamedInstantCommand(String name, Runnable toRun, Subsystem... requirements) {
        super(toRun, requirements);
        this.setName(name);
    }

    public NamedInstantCommand enableRunsWhenDisabled() {
        m_runsWhenDisabled = true;
        return this;
    }

    @Override
    public boolean runsWhenDisabled() {
        return m_runsWhenDisabled;
    }
}
