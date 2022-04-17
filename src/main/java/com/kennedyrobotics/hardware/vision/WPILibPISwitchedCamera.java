package com.kennedyrobotics.hardware.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * This helper class assumes the Raspberry PI is configured to look at the network tables key /wpilibpi/stream.
 *
 * For use with the Switched Camera on WPILibPI
 * Reference:
 * - https://github.com/wpilibsuite/WPILibPi/issues/100
 * - https://github.com/wpilibsuite/WPILibPi/commit/9fe446006814d05e28c5817669fc14c39d40fdbf
 * - https://github.com/wpilibsuite/WPILibPi/blob/main/deps/tools/multiCameraServer/src/multiCameraServer.cpp#L238-L247
 *   - Looks like both integers and strings can be used.
 * - https://www.chiefdelphi.com/t/switch-between-two-cameras-using-joystick-button-java/343521/17
 * - https://github.com/Team2470/FRC-2022-robot/blob/2022.4.17-post-10k/src/main/java/frc/robot/subsystems/Vision.java#L29-L30
 */
public class WPILibPISwitchedCamera {

    private final NetworkTable m_cameraTable = NetworkTableInstance.getDefault().getTable("wpilibpi");
    private final NetworkTableEntry m_streamSelector = m_cameraTable.getEntry("selector");
    private final int m_defaultStream;

    /**
     * Initialize a Switched Camera running on a WPILibPI. By default select stream 0.
     */
    public WPILibPISwitchedCamera() {
        this(0);
    }

    /**
     * Initialize a Switched Camera running on a WPILibPI with the provided default stream.
     *
     * @param defaultStream 0 or greater integer. Needs to be correspond to a valid selection on the PI.
     */
    public WPILibPISwitchedCamera(int defaultStream) {
        m_defaultStream = defaultStream;
        setStream(m_defaultStream);
    }

    /**
     * Set the current stream to the given index.
     * @param stream 0 or greater integer. Needs to be correspond to a valid selection on the PI.
     */
    public void setStream(int stream) {
        m_streamSelector.setDouble(stream);
    }

    /**
     * Reset the stream back to the default stream.
     */
    public void reset() {
        setStream(m_defaultStream);
    }

}
