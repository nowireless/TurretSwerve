package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Vision;

public class TurretAimToTargetCommand extends CommandBase {

    private final Vision m_vision;

    public TurretAimToTargetCommand(Vision vision) {
        m_vision = vision;
    }
}