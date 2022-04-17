package com.kennedyrobotics.hardware.motors.ctre;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPXConfiguration;
import com.ctre.phoenix.motorcontrol.can.VictorSPXPIDSetConfiguration;

public class GhostVictorSPX extends GhostBaseMotorController implements IVictorSPX {

    public GhostVictorSPX(int id) {
        super(id);
    }

    @Override
    public void set(VictorSPXControlMode mode, double value) {

    }

    @Override
    public void set(VictorSPXControlMode mode, double demand0, DemandType demand1Type, double demand1) {

    }

    @Override
    public void getPIDConfigs(VictorSPXPIDSetConfiguration pid, int pidIdx, int timeoutMs) {

    }

    @Override
    public void getPIDConfigs(VictorSPXPIDSetConfiguration pid) {

    }

    @Override
    public ErrorCode configAllSettings(VictorSPXConfiguration allConfigs, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configAllSettings(VictorSPXConfiguration allConfigs) {
        return ErrorCode.OK;
    }

    @Override
    public void getAllConfigs(VictorSPXConfiguration allConfigs, int timeoutMs) {

    }

    @Override
    public void getAllConfigs(VictorSPXConfiguration allConfigs) {

    }
}
