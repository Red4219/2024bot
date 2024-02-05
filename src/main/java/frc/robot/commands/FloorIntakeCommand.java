// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.OptionalLong;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants.kArmPoses;
import frc.robot.Constants.IntakeConstants.kIntakeStates;
import frc.robot.commands.Autonomous.IntakeCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FloorIntakeCommand extends SequentialCommandGroup {
	/** Creates a new FloorIntakeCommand. */
	public FloorIntakeCommand(boolean isIntake) {
		// Add your commands in the addCommands() call, e.g.
		// addCommands(new FooCommand(), new BarCommand());
		addCommands(
				new ArmPoseCommand(kArmPoses.GROUND_INTAKE));
				//new IntakeCommand(kIntakeStates.INTAKE, OptionalLong.empty()));
				//new ArmPoseCommand(kArmPoses.LOW_SCORE));

		if (isIntake) {
			//addCommands(new ArmPoseCommand(kArmPoses.LOW_INTAKE));
			new IntakeCommand(kIntakeStates.INTAKE, OptionalLong.empty());
		}
	}
}
