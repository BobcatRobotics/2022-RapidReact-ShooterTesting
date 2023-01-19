package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDLights extends SubsystemBase {
    private Spark LED_motor_control;

    public LEDLights(int port) {
        this.LED_motor_control = new Spark(port);
    }

    public void setLEDPower(double power) {
        this.LED_motor_control.set(power);
    }

    public double getLEDPower() {
        return this.LED_motor_control.get();
    }
    
}