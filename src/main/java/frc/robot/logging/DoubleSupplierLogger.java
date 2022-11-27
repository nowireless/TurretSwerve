package frc.robot.logging;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;

import java.util.function.DoubleSupplier;

public class DoubleSupplierLogger implements DataLogger {
        private final DoubleSupplier supplier;
        final DoubleLogEntry doubleLogger;

        public DoubleSupplierLogger(DataLog logger, String key, DoubleSupplier supplier) {
            doubleLogger = new DoubleLogEntry(logger, key);
            this.supplier = supplier;
        }

        public void periodic() {
            doubleLogger.append(supplier.getAsDouble());
        }
    }
