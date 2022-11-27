package frc.robot.subsystems;

import edu.wpi.first.hal.PowerDistributionFaults;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Power extends SubsystemBase {

    private final PowerDistribution m_pdh;

    public Power(DataLog logger) {
        m_pdh = new PowerDistribution();

        // At start up clear faults, lets see if this is a good idea or not
        m_pdh.clearStickyFaults();

        //
        // Setup logging
        //

        // Per channel current readings

    }

    public void setLimelightPower(boolean on) {
        m_pdh.setSwitchableChannel(on);
    }

    @Override
    public void periodic() {
        PowerDistributionFaults faults = m_pdh.getFaults();
    }
}
