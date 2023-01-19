package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDLights;

public class LEDControl extends CommandBase {

    private LEDLights ledLights;
    public LEDControl(LEDLights led) {
        ledLights = led;
        // SmartDashboard.putNumber("LED color", 0.99);
        addRequirements(ledLights);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        // SmartDashboard.putNumber("LED", ledLights.getLEDPower());
        // if (climber.isClimberMode()) {
        //     ledLights.setLEDPower(0.57);
        // } else if (shooter.tofTriggered()) {
        //     ledLights.setLEDPower(-0.01);
        // } else {
        //     ledLights.setLEDPower(0.99);
        // }
        // String color = SmartDashboard.getString("Team Color", "red").toLowerCase();
        // if (color.equals("red")) {
        //     ledLights.setLEDPower(0.61);
        // } else {
        //     ledLights.setLEDPower(0.87);
        // }
    }

}
