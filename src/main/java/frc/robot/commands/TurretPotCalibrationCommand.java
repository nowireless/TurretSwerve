package frc.robot.commands;

import com.team254.lib.util.CSVWritable;
import com.team254.lib.util.ReflectingCSVWriter;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

public class TurretPotCalibrationCommand extends CommandBase {

    public static class DataLine {
        public double motorAngle;
        public double potentiometerVoltage;
        public double potentiometerAngle;
    }

    private final Turret m_turret;
    private ReflectingCSVWriter<DataLine> m_csvWriter;

    public TurretPotCalibrationCommand(Turret turret) {
        m_turret = turret;

        addRequirements(m_turret);
    }

    @Override
    public void initialize() {
        m_csvWriter = new ReflectingCSVWriter<>("/home/lvuser/turret-calibration.csv", DataLine.class);
    }

    @Override
    public void execute() {
        m_turret.setPower(0.2);

        DataLine data = new DataLine();
        data.motorAngle = m_turret.getMotorAngle().getDegrees();
        data.potentiometerAngle = m_turret.getPotentiometerAngle().getDegrees();
        data.potentiometerVoltage = m_turret.getPotentiometerVoltage();


        m_csvWriter.add(data);
        m_csvWriter.flush();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_turret.setPower(0);
        m_csvWriter.flush();
        m_csvWriter = null;
    }
}
