// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants.kArmPoses;

public class ArmBumpCommand extends InstantCommand {

	private static ArmSubsystem armSubsystem;
	private double majorArmBump;
	private double minorArmBump;

	public ArmBumpCommand(double majorArmBump, double minorArmBump) {

		if(Constants.kEnableArm) {

			this.majorArmBump = majorArmBump;
			this.minorArmBump = minorArmBump;

			addRequirements(armSubsystem);
		}
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		
	}

}
