package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
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
        // if deployed(){
            
        // }
    }

    @Override
    public void end(boolean interrupted){
        climber.stop();
    }
}
