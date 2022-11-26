package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

import static frc.robot.Constants.*;

public class DriveWithController extends CommandBase {
    private static final double kDeadband = 0.15;

    private final Drivetrain drive;
    private final XboxController controller;
    private boolean fieldOrient;

    public DriveWithController(Drivetrain drive, XboxController controller) {
        this.drive = drive;
        this.controller = controller;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        fieldOrient = false;
    }

    @Override
    public void execute() {
        double xMove = -controller.getLeftY();
        double yMove = -controller.getLeftX();
        double rotate;

        double left = controller.getLeftTriggerAxis();
        double right = controller.getRightTriggerAxis();

        if (left < right) {
            rotate =  -right;
        } else {
            rotate =  left;
        }

        if (Math.abs(xMove) < kDeadband) {
            xMove = 0;
        }

        if (Math.abs(yMove) < kDeadband) {
            yMove = 0;
        }

        if (Math.abs(rotate) < kDeadband) {
            rotate = 0;
        }

        xMove = Math.copySign(xMove * xMove, xMove);
        yMove = Math.copySign(yMove * yMove, yMove);
        rotate = Math.copySign(rotate * rotate, rotate);

        // TODO add an option, to easily nerf in this command
//        xMove *= 0.3;
//        yMove *= 0.3;
//        rotate *= 0.25;
        xMove *= 0.8;
        yMove *= 0.8;
        rotate *= 0.5;

        if(controller.getAButton()) {
            fieldOrient = false;
        } else if (controller.getBButton()) {
            fieldOrient = true;
        }

        // Now we need to map the percentages to Meters (or Radians) per second, as that is what the drive train
        // subsystem accepts
        xMove *= ModuleConstants.kMaxDriveVelocityMetersPerSecond;
        yMove *= ModuleConstants.kMaxDriveVelocityMetersPerSecond;
        rotate *= DriveConstants.kMaxAngularVelocityRadiansPerSecond;

        SmartDashboard.putNumber("Drive - xMove", xMove);
        SmartDashboard.putNumber("Drive - yMove", yMove);
        SmartDashboard.putNumber("Drive - rotate", rotate);

        drive.drive(xMove, yMove, rotate, fieldOrient);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }

}