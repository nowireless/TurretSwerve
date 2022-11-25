package frc.robot.subsystems;

import edu.wpi.first.hal.PowerDistributionFaults;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.logging.DataLogger;

import java.util.function.DoubleSupplier;

public class Power extends SubsystemBase {

    private final PowerDistribution pdh;

    public Power(DataLog logger) {
        pdh = new PowerDistribution();

        // At start up clear faults, lets see if this is a good idea or not
        pdh.clearStickyFaults();

        //
        // Setup logging
        //

        // Per channel current readings

    }

    public void setLimelightPower(boolean on) {
        pdh.setSwitchableChannel(on);
    }

    @Override
    public void periodic() {
        PowerDistributionFaults faults = pdh.getFaults();
    }
}
