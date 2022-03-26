package frc.robot.subsystems;

import edu.wpi.first.wpilibj.I2C.Port;
import com.revrobotics.ColorSensorV3;

public class ColorSensor {
    private ColorSensorV3 colorSensor;
    private LEDLights lights;

    public ColorSensor(Port port) {
        colorSensor = new ColorSensorV3(port);
        lights = new LEDLights(/* port number */);
    }
    
    // returns color of current ball, either "red" or "blue"
    // returns null if no ball present
    public String getCurrentBall() {
        if (ballPresent() && colorSensor.getBlue() > colorSensor.getRed()) {
            return "blue";
        } else if (ballPresent() && colorSensor.getRed() > colorSensor.getBlue()) {
            return "red";
        }
        return null;
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
        if (getCurrentBall().equals("blue")) {
            // turns lights blue
            lights.setLEDPower(0.87);
        } else if (getCurrentBall().equals("red")) {
            // turns lights red
            lights.setLEDPower(0.61);
        } else {
            // turns lights gray
            lights.setLEDPower(0.95);
        }
    }
    

}
