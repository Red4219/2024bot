package frc.robot.subsystems;

import java.util.Date;
import java.util.Timer;
import java.util.TimerTask;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants.kShooterStates;
import frc.robot.Mechanisms.ShooterWheels;

public class ShooterSubsystem extends SubsystemBase {
    
    private ShooterWheels shooter = new ShooterWheels(Constants.ShooterConstants.kPrimaryPort, Constants.ShooterConstants.kSecondaryPort);
    private kShooterStates currentShooterState = kShooterStates.DISABLED;

    public ShooterSubsystem() {
        
    }

    @Override
	public void periodic() {

    }

    public void shootNote() {
        shooter.shoot(1.0);
	}

	public void disableShooter() {
        shooter.disable();
	}

	public void setShooterState(kShooterStates state) {

		currentShooterState = state;

		switch (currentShooterState) {

			case IDLE:
				disableShooter();
				break;

			case SHOOT:
				shootNote();
				break;

			case DISABLED:
				disableShooter();
				break;

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

    public InstantCommand shootCommand() {
		return new InstantCommand(() -> setShooterState(kShooterStates.SHOOT));
	}

    public InstantCommand idleCommand() {
		return new InstantCommand(() -> setShooterState(kShooterStates.IDLE));
	}

    public InstantCommand disableCommand() {
		return new InstantCommand(() -> setShooterState(kShooterStates.DISABLED));
	}

    public void setSpeed(double speed) {
        System.out.println("ShooterSubSystem::setSpeed() - speed: " + speed);
    }

    public double getSpeed() {
        return shooter.getSpeed();
    }

    /*public InstantCommand timedShootCommand(long timeToShoot) {
        return new InstantCommand(() -> shootTime(timeToShoot));
    }*/
}
