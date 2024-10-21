package frc.robot.subsystems;

import java.util.Date;
import java.util.Timer;
import java.util.TimerTask;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants.kShooterStates;
import frc.robot.Mechanisms.ShooterWheels;

public class ShooterSubsystem extends SubsystemBase {
    
    private ShooterWheels shooter;
    private kShooterStates currentShooterState = kShooterStates.STOPPED;
    private double speed = 0.0;
    private String status = "STOPPED";
    ArmSubsystem armSubsystem;

    public ShooterSubsystem(ArmSubsystem armSubsystem) {

        if (Constants.kEnableArm) {
            this.armSubsystem = armSubsystem;
            shooter = new ShooterWheels(Constants.ShooterConstants.kPrimaryPort, Constants.ShooterConstants.kSecondaryPort);

            if (Constants.debugShooter == true) {
                ShuffleboardTab shooterTab = Shuffleboard.getTab("Shooter");
                shooterTab.addDouble("Target Speed", this::getSpeed);
                shooterTab.addString("Status", this::getStatus);
            }
        }
    }

    @Override
	public void periodic() {

    }

    public void shootSpeakerNote() {
        if (Constants.kEnableArm) {
            shooter.shoot(Constants.ShooterConstants.kSpeakerShootSpeed);
        }
	}

    public void shootAmpNote() {
        if (Constants.kEnableArm) {
            shooter.shoot(Constants.ShooterConstants.kAmpShootSpeed);
        }
	}

    public void stopShooter() {
        if (Constants.kEnableArm) {
            shooter.stop();
        }
    }

	public void disableShooter() {
        if (Constants.kEnableArm) {
            shooter.disable();
        }
	}

	public void setShooterState(kShooterStates state) {

        if (Constants.kEnableArm) {

            if (currentShooterState == state) {
                stopShooter();
                currentShooterState = kShooterStates.STOPPED;
                this.status = "STOPPED";
            } else {

                currentShooterState = state;

                switch (currentShooterState) {
                    case IDLE:
                        disableShooter();
                        this.status = "IDLE";
                        break;

                    case SHOOT_SPEAKER:
                        shootSpeakerNote();
                        this.status = "SHOOT_SPEAKER";
                        break;

                    case SHOOT_AMP:
                        shootAmpNote();
                        this.status = "SHOOT_AMP";
                        break;

                    case STOPPED:
                        stopShooter();
                        this.status = "STOPPED";
                        break;

                    case DISABLED:
                        disableShooter();
                        this.status = "DISABLED";
                        break;
                }
            }
        }
    }

    public InstantCommand shootSpearkerCommand() {
		return new InstantCommand(() -> setShooterState(kShooterStates.SHOOT_SPEAKER));
	}

    public InstantCommand idleCommand() {
		return new InstantCommand(() -> setShooterState(kShooterStates.IDLE));
	}

    public InstantCommand disableCommand() {
		return new InstantCommand(() -> setShooterState(kShooterStates.DISABLED));
	}

    public void setSpeed(double speed) {
        this.speed = speed;
    }

    public double getSpeed() {
        return shooter.getSpeed();
    }

    public String getStatus() {
        return this.status;
    }
}
