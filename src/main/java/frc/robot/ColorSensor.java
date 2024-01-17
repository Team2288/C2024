package frc.robot;

import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;
import edu.wpi.first.wpilibj.I2C;

public class ColorSensor {
    I2C.Port i2cPort;
    ColorSensorV3 colorSensor;
    ColorMatch colorMatcher;
    Color notecolor;

    public ColorSensor() {
        i2cPort = I2C.Port.kOnboard;

        colorSensor = new ColorSensorV3(i2cPort);
        colorMatcher = new ColorMatch();

        notecolor = new Color("#FF6600");
        colorMatcher.addColorMatch(notecolor);
    }   

    public boolean getNoteDetected() {
        Color detectedcolor = colorSensor.getColor();
        ColorMatchResult match = colorMatcher.matchClosestColor(detectedcolor);

        return match.color.equals(notecolor);
    }
}
