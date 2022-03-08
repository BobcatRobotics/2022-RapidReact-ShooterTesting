package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

public class ClimberCommand extends CommandBase {
    
    private final Climber climber;
    private final Joystick rightStick;
    private final Joystick gp;
    private boolean deployed = false;


    public ClimberCommand(Climber c, Joystick r, Joystick gp) {
        this.climber = c;
        this.rightStick = r;
        this.gp = gp;
    }

    /*

    - if deployed then do work otherwise idc
        deploy button = left bumper 
        while the joystick is active then pull in abs of r stick val
        
    */
    @Override
    public void execute() {

        // Check if currently in climber mode
        if (climber.isClimberMode()) {
            climber.climb(gp.getRawAxis(Constants.Gamepad_Left_Joystick_Y_Axis));
        }
        // If not on climber mode, stop the climber
        else {
            climber.stop();
        }

        // // D-pad left -> winch motor rewind
        // if (gp.getPOV() == Constants.D_Pad_Left && climber.isClimberMode()) {
        //     climber.climb(false);
        // }
        // // D-pad right -> winch motor unwind
        // else if (gp.getPOV() == Constants.D_Pad_Right && climber.isClimberMode()) {
        //     climber.climb(true);
        // }
        // // Otherwise, stop moving motor
        // else {
        //     climber.stop();
        // }

        // X button -> climber solenoid withdraw
        if (gp.getRawButton(Constants.X_Button) && climber.isClimberMode()) {
            // // System.out.println("X pressed");
            if (climber.isDeployed()) {
                // // System.out.println("X pressed - climber solenoid withdraw");
                climber.withdraw();
            }
        }
        // Y button -> climber solenoid deploy
        if (gp.getRawButton(Constants.Y_Button) && climber.isClimberMode()) {
            // // System.out.println("Y pressed");
            if (!climber.isDeployed()) {
                // // System.out.println("Y pressed - climber solenoid deployed");
                climber.deploy();
            }
        }
    }

    @Override
    public void end(boolean interrupted){
        climber.stop();
    }
}
