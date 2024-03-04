package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Tools.PhotonVision;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AimCommand extends Command {

    private static DriveSubsystem _driveSubsystem;
    private static ArmSubsystem _armSubsystem;
    private static ShooterSubsystem _shooterSubsystem;
    private PhotonVision _photonVision;
    private int _targedNumber = 7;

    public AimCommand(PhotonVision photonVision) {
        _driveSubsystem = RobotContainer.driveSubsystem;
        _armSubsystem = RobotContainer.armSubsystem;
        _shooterSubsystem = RobotContainer.shooterSubsystem;
        // Use addRequirements() here to declare subsystem dependencies.
		addRequirements(_driveSubsystem, _armSubsystem, _shooterSubsystem);

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

    public double calculateShooterHeight(double distance) {
        return distance;
    }

    public double calculateShooterSpeed(double distance) {
        return distance;
    }

    @Override
    public boolean isFinished() {

        if(_photonVision.canSeeTarget(_targedNumber) == false) {
            System.out.println("AimCommand::isFinished() - cannot see the target so returning true");
            // Stop because we cannot see the target
            return true;
        } 

        double targetYaw = _photonVision.aimAtTarget(_targedNumber);
        double shooterHeight = calculateShooterHeight(_photonVision.targetDistance(_targedNumber));
        double shooterSpeed = calculateShooterSpeed(_photonVision.targetDistance(_targedNumber));

        //System.out.println("isFinished() - targetYaw: " + targetYaw + " targetDistance: " + targetDistance);

        _armSubsystem.moveToPosition(shooterHeight);
        _shooterSubsystem.setSpeed(shooterSpeed);

        double position = _armSubsystem.getPosition();
        double speed = _shooterSubsystem.getSpeed();

        if(targetYaw == 0.0) {
            return true;
        }

        if(Math.abs(targetYaw) > Constants.AutoConstants.kAimTargetTolerance) {

            if(targetYaw > 0) {
                _driveSubsystem.drive(0.0, 0.0, -.1);
                //_driveSubsystem.drive(0.0, 0.0, .5);
            } else {
                _driveSubsystem.drive(0.0, 0.0, .5);
                //_driveSubsystem.drive(0.0, 0.0, -.1);
            }
            
            return false;
        } 
        
        return true;
    }
    
}
