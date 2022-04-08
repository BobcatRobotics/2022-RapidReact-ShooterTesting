package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDLights;
import frc.robot.subsystems.Shooter;

public class LEDControl extends CommandBase {

    private LEDLights ledLights;
    private Joystick gamepad;
    private Drivetrain drivetrain;
    private Intake intake;
    private Shooter shooter;
    private Climber climber;
    
    public LEDControl(LEDLights led, Shooter sh, Climber cb) {
        ledLights = led;
        shooter = sh;
        climber = cb;
        SmartDashboard.putNumber("LED color", 0.59);
        addRequirements(ledLights);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (climber.isClimberMode()) {
            ledLights.setLEDPower(0.57);
        } else if (shooter.tofTriggered()) {
            ledLights.setLEDPower(-0.01);
        } else {
            ledLights.setLEDPower(SmartDashboard.getNumber("LED color", 0.59));
        }
    }

}
