package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Tools.PhotonVision;
import frc.robot.subsystems.DriveSubsystem;

public class AimCommand extends Command {

    private static DriveSubsystem _driveSubsystem;
    private PhotonVision _photonVision;
    private int _targedNumber = 7;

    public AimCommand(PhotonVision photonVision) {
        _driveSubsystem = RobotContainer.driveSubsystem;
        // Use addRequirements() here to declare subsystem dependencies.
		addRequirements(_driveSubsystem);

        _photonVision = photonVision;
    }

    @Override
    public void initialize() {
        if(DriverStation.getAlliance().isPresent()) {
            if(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
                _targedNumber = 7;
            } else {
                _targedNumber = 4;
            }
        }
    }

    @Override
    public void execute() {
        
        /*if(_photonVision.hasTarget()) {
            _isTargeted = _photonVision.aimAtTarget(7); // look at the blue target
        }*/
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {

        if(_photonVision.canSeeTarget(_targedNumber) == false) {
            // Stop because we cannot see the target
            return true;
        } 

        double targetYaw = _photonVision.aimAtTarget(_targedNumber);

        System.out.println("isFinished() - targetYaw: " + targetYaw);

        if(targetYaw == 0.0) {
            return true;
        }

        if(Math.abs(targetYaw) > Constants.AutoConstants.kAimTargetTolerance) {

            if(targetYaw > 0) {
                _driveSubsystem.drive(0.0, 0.0, -.1);
            } else {
                _driveSubsystem.drive(0.0, 0.0, .5);
            }
            
            return false;
        } 
        
        return true;
    }
    
}
