package frc.robot.subsystems;

import static frc.robot.Constants.LEDLightsConstants.*;

import edu.wpi.first.wpilibj.motorcontrol.*;

public class LEDLights {
    private Spark LED_motor_control;

    public LEDLights() {
        this.LED_motor_control = new Spark(ledPort);
    }

    public void setLEDPower(double power) {
        this.LED_motor_control.set(power);
    }

    public double getLEDPower() {
        return this.LED_motor_control.get();
    }

}
