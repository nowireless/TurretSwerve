package com.kennedyrobotics.hardware.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.util.Color;

import java.util.Optional;

/**
 * Access and control a Limelight via NetworkTables.
 * Based on the API documented here: https://docs.limelightvision.io/en/latest/networktables_api.html
 *
 * TODOs
 * - 3d pose data from camtran
 * - Python - https://docs.limelightvision.io/en/latest/networktables_api.html#python
 * - Corners - https://docs.limelightvision.io/en/latest/networktables_api.html#corners
 * - Raw Targets - https://docs.limelightvision.io/en/latest/networktables_api.html#advanced-usage-with-raw-contours
 */
public class Limelight {
    public enum StreamMode {
        kSideBySide(0),
        kLimelightPrimary(1),
        kWebcamPrimary(2);
        public final int value;
        StreamMode(int value) { this.value = value; }
    }

    public enum LEDMode {
        kPipelineDefault(0),
        kOff(1),
        kForceBlink(2),
        kOn(3);
        public final int value;
        LEDMode(int value) { this.value = value; }
    }

    public enum ProcessingMode {
        kPipeline(0),
        kDriver(1);
        public final int value;
        ProcessingMode(int value) { this.value = value; }
    }

    private final NetworkTable m_limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    // Basic targetting data
    private final NetworkTableEntry m_tv = m_limelightTable.getEntry("tv");
    private final NetworkTableEntry m_tx = m_limelightTable.getEntry("tx");
    private final NetworkTableEntry m_ty = m_limelightTable.getEntry("ty");
    private final NetworkTableEntry m_ta = m_limelightTable.getEntry("ta");
    private final NetworkTableEntry m_ts = m_limelightTable.getEntry("ts");
    private final NetworkTableEntry m_tl = m_limelightTable.getEntry("tl");
    private final NetworkTableEntry m_tshort = m_limelightTable.getEntry("tshort");
    private final NetworkTableEntry m_tlong = m_limelightTable.getEntry("tlong");
    private final NetworkTableEntry m_thor = m_limelightTable.getEntry("thor");
    private final NetworkTableEntry m_tvert = m_limelightTable.getEntry("tvert");
    private final NetworkTableEntry m_getpipe = m_limelightTable.getEntry("getpipe");
    private final NetworkTableEntry m_json = m_limelightTable.getEntry("json");
    private final NetworkTableEntry m_tclass = m_limelightTable.getEntry("tclass");
    private final NetworkTableEntry m_tc = m_limelightTable.getEntry("tc");

    // Configuration
    private final NetworkTableEntry m_pipeline = m_limelightTable.getEntry("pipeline");
    private final NetworkTableEntry m_snapshot = m_limelightTable.getEntry("snapshot");
    private final NetworkTableEntry m_crop = m_limelightTable.getEntry("crop");


    public void setStreamMode(StreamMode mode) {
        m_limelightTable.getEntry("stream").setNumber(mode.value);
    }

    public void setProcessingMode(ProcessingMode mode) {
        m_limelightTable.getEntry("camMode").setNumber(mode.value);
    }

    public void setLEDMode(LEDMode mode) {
        m_limelightTable.getEntry("ledMode").setNumber(mode.value);
    }

    /**
     * Whether the limelight has any valid targets
     * @return true if the target is present
     */
    public boolean isTargetPresent() {
        return m_tv.getDouble(0.0) == 1.0;
    }

    /**
     * Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27 degrees | LL2: -29.8 to 29.8 degrees)
     * @return if the target is present, the horizontal offset of the target. Otherwise nothing.
     */
    public Optional<Rotation2d> getTargetHorizontalAngle() {
        if (!isTargetPresent()) return Optional.empty();

        return Optional.of(
            Rotation2d.fromDegrees(m_tx.getDouble(0.0))
        );
    }

    /**
     * Vertical Offset From Crosshair To Target (LL1: -20.5 degrees to 20.5 degrees | LL2: -24.85 to 24.85 degrees)
     * @return if the target is present, the Vertical offset of the target. Otherwise nothing.
     */
    public Optional<Rotation2d> getTargetVerticalAngle() {
        if (!isTargetPresent()) return Optional.empty();

        return Optional.of(
            Rotation2d.fromDegrees(m_ty.getDouble(0.0))
        );
    }

    /**
     * Target Area (0% of image to 100% of image)
     * @return if the target is present, the target area, otherwise nothing.
     */
    public Optional<Double> getTargetArea() {
        if (!isTargetPresent()) return Optional.empty();

        return Optional.of(
            m_ta.getDouble(0.0)
        );
    }

    /**
     * Target Skew or rotation (-90 degrees to 0 degrees)
     * @return if the target is present, the target skew
     */
    public Optional<Rotation2d> getTargetSkewOrRotation() {
        if (!isTargetPresent()) return Optional.empty();

        return Optional.of(
            Rotation2d.fromDegrees(m_ts.getDouble(0.0))
        );
    }

    /**
     * Sidelength of shortest side of the fitted bounding box (pixels)
     * @return if target present, the length in pixels. Otherwise nothing. If network table issue -1
     */
    public Optional<Double> getTargetFittedBoundingSideLengthShort() {
        if (!isTargetPresent()) return Optional.empty();

        return Optional.of(
            m_tshort.getDouble(-1)
        );
    }

    /**
     * Sidelength of longest side of the fitted bounding box (pixels)
     * @return if target present, the length in pixels. Otherwise nothing. If network table issue -1
     */
    public Optional<Double> getTargetFittedBoundingSideLengthLong() {
        if (!isTargetPresent()) return Optional.empty();

        return Optional.of(
            m_tlong.getDouble(-1)
        );
    }

    /**
     * Horizontal sidelength of the rough bounding box (0 - 320 pixels)
     * @return if target present, the length in pixels. Otherwise nothing. If network table issue -1
     */
    public Optional<Double> getTargetRoughBoundingBoxHorizontalLength() {
        if (!isTargetPresent()) return Optional.empty();

        return Optional.of(
            m_thor.getDouble(-1)
        );
    }

    /**
     * Vertical sidelength of the rough bounding box (0 - 320 pixels)
     * @return if target present, the length in pixels. Otherwise nothing. If network table issue -1
     */
    public Optional<Double> getTargetRoughBoundingBoxVerticalLength() {
        if (!isTargetPresent()) return Optional.empty();

        return Optional.of(
            m_tvert.getDouble(-1)
        );
    }

    /**
     * Class ID of primary neural detector result or neural classifier result
     * @return if target present, the class ID of the target
     */
    public Optional<Integer> getTargetClass() {
        if (!isTargetPresent()) return Optional.empty();

        return Optional.of(
            (int)m_tclass.getDouble(-1)
        );
    }

    /**
     * Full JSON dump of targeting results
     * @return
     */
    public String getJSONRaw() {
        return m_json.getString("null");
    }

    /**
     * Get the average HSV color underneath the crosshair region as a NumberArray
     * @return The color under the cross hair
     */
    public Color getCrossHairAverageColor() {
        var hsvColor = m_tc.getNumberArray(new Number[]{0,0,0});

        return Color.fromHSV((int)hsvColor[0], (int)hsvColor[1], (int)hsvColor[2]);
    }

    /**
     * The pipeline’s latency contribution (ms) Add at least 11ms for image capture latency.
     * @return latency in ms
     */
    public double getPipelineLatencyMS() {
        return m_tl.getDouble(-1);
    }

    /**
     * True active pipeline index of the camera (0 .. 9)
     * @return if key is present 0 to 9, otherwise -1
     */
    public int getActivePipelineIndex() {
        return (int) m_getpipe.getDouble(-1);
    }

    /**
     * Sets limelight’s current pipeline
     * @param index Select pipeline 0..9
     */
    public void setPipelineIndex(int index) {
        m_pipeline.setDouble(index);
    }

    /**
     * Allows users to take snapshots during a match, reset snapshot mode.
     */
    public void resetSnapshot() {
        m_snapshot.setDouble(0);
    }

    /**
     * Allows users to take snapshots during a match, take exactly one snapshot
     */
    public void takeSnapshot() {
        m_snapshot.setDouble(1);
    }

    /**
     * Sets the crop rectangle. The pipeline must utilize the default crop rectangle in the web interface. The array must have exactly 4 entries.
     *
     * @param x0 [0] X0 - Min or Max X value of crop rectangle (-1 to 1)
     * @param x1 [1] X1 - Min or Max X value of crop rectangle (-1 to 1)
     * @param y0 [2] Y0 - Min or Max Y value of crop rectangle (-1 to 1)
     * @param y1 [3] Y1 - Min or Max Y value of crop rectangle (-1 to 1)
     */
    public void setCrop(double x0, double x1, double y0, double y1) {
        m_crop.setDoubleArray(new double[]{x0, x1, y0, y1});
    }


}
