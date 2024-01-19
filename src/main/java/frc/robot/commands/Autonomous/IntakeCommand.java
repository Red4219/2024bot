// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import java.util.OptionalLong;
import java.util.Timer;
import java.util.TimerTask;

import edu.wpi.first.math.util.Units;
//import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.RobotContainer;
import frc.robot.Constants.IntakeConstants.kIntakeStates;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {

	private IntakeSubsystem intakeSubsystem;
	private OptionalLong intakeTime = OptionalLong.empty();
	private boolean finished = false;
	private kIntakeStates intakeState;

	//private Timer timer;
	//private double timeMilis;
	//private boolean suck;

	public IntakeCommand(IntakeSubsystem intakeSubsystem, kIntakeStates state, OptionalLong intakeTime) {

		//intakeSubsystem = RobotContainer.intakeSubsystem;

		//timer = new Timer();
		//this.timeMilis = timeMilis;
		//this.suck = suck;

		this.intakeSubsystem = intakeSubsystem;
		this.intakeTime = intakeTime;
		this.intakeState = state;

		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(intakeSubsystem);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		intakeSubsystem.setIntakeState(intakeState);

		if(intakeTime.isPresent()) {

            TimerTask task = new TimerTask() {
                public void run() {
                    System.out.println("stopping the intake");
                     finished = true;
                }
            };
            Timer timer = new Timer("Timer");
    
            timer.schedule(task, intakeTime.getAsLong());
            System.out.println("starting the intake");
            finished = false;
        } else {
            finished = true;
        }


		//timer.restart();
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		/*if (suck) {
			intakeSubsystem.setIntakeState(kIntakeStates.INTAKE);
		} else {
			intakeSubsystem.setIntakeState(kIntakeStates.OUTTAKE);
		}*/

		if(intakeSubsystem.noteDetected()) {
			finished = true;
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		//intakeSubsystem.startSucking();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		//return (Units.secondsToMilliseconds(timer.get()) >= timeMilis);
		return finished;
	}
}
