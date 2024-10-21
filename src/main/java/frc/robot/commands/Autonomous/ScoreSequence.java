// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants.kArmPoses;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class ScoreSequence extends SequentialCommandGroup {

	public static IntakeSubsystem intakeSubsystem;
	public static ArmSubsystem armSubsystem;
 
	/** Creates a new ScoreCommand. */
	public ScoreSequence(kArmPoses armPose) {

	}
}
