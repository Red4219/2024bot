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
    
    private ShooterWheels shooter = new ShooterWheels(Constants.ShooterConstants.kPrimaryPort, Constants.ShooterConstants.kSecondaryPort);
    private kShooterStates currentShooterState = kShooterStates.DISABLED;
    private double speed = 0.0;
    private String status = "STOPPED";

    public ShooterSubsystem() {

        if(Constants.debugShooter == true) {
            ShuffleboardTab shooterTab = Shuffleboard.getTab("Shooter");
		    shooterTab.addDouble("Target Speed", this::getSpeed);
            shooterTab.addString("Status", this::getStatus);
        }
        
    }

    @Override
	public void periodic() {

    }

    public void shootSpeakerNote() {
        shooter.shoot(0.5);
	}

    public void shootAmpNote() {
        shooter.shoot(0.1);
	}

    public void stopShooter() {
        shooter.stop();
    }

	public void disableShooter() {
        shooter.disable();
	}

	public void setShooterState(kShooterStates state) {

        if(currentShooterState == state) {
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

    /*public void shootTime(long timeToShoot) {
        System.out.println("called");


        TimerTask task = new TimerTask() {
            public void run() {

                setShooterState(kShooterStates.IDLE);
                System.out.println("stopping the shooter");
            }
        };
        Timer timer = new Timer("Timer");
    
        timer.schedule(task, timeToShoot);

        setShooterState(kShooterStates.SHOOT);
        System.out.println("starting the shooter");
        
    }*/

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
        //System.out.println("ShooterSubSystem::setSpeed() - speed: " + speed);
    }

    public double getSpeed() {
        return shooter.getSpeed();
    }

    public String getStatus() {
        return this.status;
    }

    /*public InstantCommand timedShootCommand(long timeToShoot) {
        return new InstantCommand(() -> shootTime(timeToShoot));
    }*/
}
