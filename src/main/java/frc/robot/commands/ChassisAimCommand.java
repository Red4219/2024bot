package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants.kDriveModes;
import frc.robot.subsystems.DriveSubsystem;

// This will point the chassis at the speaker while driving

public class ChassisAimCommand extends Command {
    private static DriveSubsystem _driveSubsystem;

    public ChassisAimCommand() {
        _driveSubsystem = RobotContainer.driveSubsystem;
        addRequirements(_driveSubsystem);
    }

    @Override
	public void initialize() {
        // If we are already aiming, set the arm to idle to stop
        if(_driveSubsystem.getMode() == kDriveModes.AIM) {
            _driveSubsystem.setMode(kDriveModes.NORMAL);
        } else {
            _driveSubsystem.setMode(kDriveModes.AIM);

        }
    }

    @Override
	public void execute() {
        
	}

    @Override
	public void end(boolean interrupted) {
	}

    @Override
	public boolean isFinished() {
        return true;
	}
    
}
