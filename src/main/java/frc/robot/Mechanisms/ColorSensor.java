package frc.robot.Mechanisms;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorSensorV3.ColorSensorMeasurementRate;
import com.revrobotics.ColorSensorV3.ColorSensorResolution;
import com.revrobotics.ColorSensorV3.GainFactor;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;

import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;

public class ColorSensor {
    private final I2C.Port i2cPort = I2C.Port.kOnboard;
    private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
    private final ColorMatch m_colorMatcher = new ColorMatch();

    private final Color kBlueTarget = new Color(0.143, 0.427, 0.429);
    private final Color kGreenTarget = new Color(0.197, 0.561, 0.240);
    private final Color kRedTarget = new Color(0.561, 0.232, 0.114);
    private final Color kYellowTarget = new Color(0.361, 0.524, 0.113);

    public void init() {
        m_colorMatcher.addColorMatch(kBlueTarget);
        m_colorMatcher.addColorMatch(kGreenTarget);
        m_colorMatcher.addColorMatch(kRedTarget);
        m_colorMatcher.addColorMatch(kYellowTarget); 

        m_colorSensor.configureColorSensor(ColorSensorResolution.kColorSensorRes13bit, ColorSensorMeasurementRate.kColorRate500ms, GainFactor.kGain1x);
    }

    public Color getColor() {
        //Color detectedColor = m_colorSensor.getColor();
        return m_colorSensor.getColor();
    }

    /*public void printColorValues() {
        System.out.println("Blue: " + m_colorSensor.getBlue() + " Red: " + m_colorSensor.getRed() + " Green: " + m_colorSensor.getGreen());
    }*/

    public boolean noteDetected() {
        if(
            m_colorSensor.getRed() >= Constants.IntakeConstants.kColorSensorGreaterThanRed
            && m_colorSensor.getBlue() <= Constants.IntakeConstants.kColorSensorLessThanBlue
            && m_colorSensor.getGreen() <= Constants.IntakeConstants.kColorSensorLessThanGreen
        ) {
            //System.out.println("note detected");
            return true;
        }

        return false;
    }

    public int getBlue() {
        return m_colorSensor.getBlue();
    }

    public int getRed() {
        return m_colorSensor.getRed();
    }

    public int getGreen() {
        return m_colorSensor.getGreen();
    }
}
