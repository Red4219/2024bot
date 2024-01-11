package frc.robot.Tools;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.util.Units;
import frc.robot.Tools.GyroIO.GyroIOInputs;

public class GyroIONavX {
	private final double[] xyzDps = new double[3];
    //private final AHRS gyro = new AHRS(SPI.Port.kMXP);
    private final AHRS gyro;

    public GyroIONavX(AHRS gyroscope) {
      this.gyro = gyroscope;
    }
    
    
    public void updateInputs(GyroIOInputs inputs) {
        // Again, the 2022 code base has good examples for reading this data. We generally prefer to use
        // "getAngle" instead of "getYaw" (what's the difference?)
        //
        // Remember to pay attention to the UNITS.
        xyzDps[0] = gyro.getRawGyroX();
        xyzDps[1] = gyro.getRawGyroY();
        xyzDps[2] = -gyro.getRawGyroZ();
        inputs.connected = gyro.isConnected();
        inputs.positionRad = Units.degreesToRadians(gyro.getYaw());
        inputs.velocityRadPerSec = Units.degreesToRadians(xyzDps[2]);
		inputs.positionDegrees = gyro.getYaw();
        inputs.x = xyzDps[0];
        inputs.y = xyzDps[1];
        inputs.z = xyzDps[2];
      }
}
