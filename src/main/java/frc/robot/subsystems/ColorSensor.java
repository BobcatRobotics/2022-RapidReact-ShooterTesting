package frc.robot.subsystems;

import static frc.robot.Constants.ColorSensorConstants.*;

import com.revrobotics.ColorSensorV3;

public class ColorSensor {
    private ColorSensorV3 colorSensor;
    private LEDLights lights;

    public ColorSensor() {
        colorSensor = new ColorSensorV3(colorSensorPort);
        lights = new LEDLights();
    }
    
    // Returns color of current ball, either "red" or "blue"
    // Returns null if no ball present
    public String getBallColor() {
        if (ballPresent() && colorSensor.getBlue() > colorSensor.getRed()) {
            return "blue";
        } else if (ballPresent() && colorSensor.getRed() > colorSensor.getBlue()) {
            return "red";
        }
        return null;
    }

    /**
     * Returns the proximity from the color sensor
     * @return int between 0 and 2047
     */
    public int getProximity() {
        return colorSensor.getProximity();
    }

    public boolean ballPresent() {
        // getProximity returns a value between 0 and 2047
        // 1700 is just a temporary number, I have not tested it yet
        if (colorSensor.getProximity() > 1700) {
            return true;
        }
        return false;
    }

    // Sets LED color based on ball data
    public void setLEDS() {
        if (getBallColor().equals("blue")) {
            // turns lights blue
            lights.setLEDPower(0.87);
        } else if (getBallColor().equals("red")) {
            // turns lights red
            lights.setLEDPower(0.61);
        } else {
            // turns lights gray
            lights.setLEDPower(0.95);
        }
    }
    

}
