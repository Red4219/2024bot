package frc.robot.Mechanisms;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants;
import frc.robot.Constants.Mode;

public class ShooterWheels {
    private CANSparkMax _shooterSparkMaxPrimary;
    private CANSparkMax _shooterSparkMaxSecondary;
    private RelativeEncoder rightEncoder, leftEncoder;
    private double targetSpeed = 0.0;

    public ShooterWheels(int primaryPort, int secondaryPort) {

        // Primary Motor

        _shooterSparkMaxPrimary = new CANSparkMax(primaryPort, MotorType.kBrushless);

        // Setup simulation
        if(Constants.getMode() == Mode.SIM) {
			REVPhysicsSim.getInstance().addSparkMax(_shooterSparkMaxPrimary, 2.6f, 5676);
		}

        _shooterSparkMaxPrimary.clearFaults();
		_shooterSparkMaxPrimary.restoreFactoryDefaults();
		_shooterSparkMaxPrimary.setIdleMode(IdleMode.kBrake);
		_shooterSparkMaxPrimary.setSmartCurrentLimit(Constants.ShooterConstants.kSmartCurrentLimit);

        _shooterSparkMaxSecondary = new CANSparkMax(secondaryPort, MotorType.kBrushless);

        // Secondary motor

        // Setup simulation
        if(Constants.getMode() == Mode.SIM) {
			REVPhysicsSim.getInstance().addSparkMax(_shooterSparkMaxSecondary, 2.6f, 5676);
		}

        _shooterSparkMaxSecondary.clearFaults();
		_shooterSparkMaxSecondary.restoreFactoryDefaults();
		_shooterSparkMaxSecondary.setIdleMode(IdleMode.kBrake);
		_shooterSparkMaxSecondary.setSmartCurrentLimit(Constants.ShooterConstants.kSmartCurrentLimit);

        _shooterSparkMaxSecondary.follow(_shooterSparkMaxPrimary, true);

        rightEncoder = _shooterSparkMaxPrimary.getEncoder();
		leftEncoder = _shooterSparkMaxSecondary.getEncoder();
    }

    public void disable() {
		//System.out.println("ShooterWheels::disable() called");

		if(Constants.getMode() == Mode.SIM) {
			_shooterSparkMaxPrimary.setVoltage(0.0);
            _shooterSparkMaxSecondary.setVoltage(0.0);
		} else {
			_shooterSparkMaxPrimary.stopMotor();
		}
	}

    public void shoot(double speed) {
        targetSpeed = speed;
        if(Constants.getMode() == Mode.SIM) {
			_shooterSparkMaxPrimary.setVoltage(speed);
            _shooterSparkMaxSecondary.setVoltage(speed);
		} else {
			_shooterSparkMaxPrimary.set(speed);
		}
    }

    public double getSpeed() {
        return _shooterSparkMaxPrimary.get();
    }
    
}
