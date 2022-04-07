package frc.robot.commands.five_ball_auto_commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class FV_IntakeDownAndSuck extends CommandBase {
    
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private Intake m_intake = null;

    public FV_IntakeDownAndSuck(Intake intake) {
      m_intake = intake;
      addRequirements(intake);
    }
  
    @Override
    public void initialize() {
        //Drop the intake
        if(!m_intake.isDeployed()){
            m_intake.deploy(true);
        }
    }
  
    @Override
    public void execute() {
        //Run the Intake
        m_intake.feedIn();
    }
  
    @Override
    public void end(boolean interrupted) {
        //Stop the intake
    }
  
    @Override
    public boolean isFinished() {
      return m_intake.isDeployed();
    }

}
