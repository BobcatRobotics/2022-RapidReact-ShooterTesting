package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class dropAndSuck extends CommandBase{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private Intake m_intake = null;
    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public dropAndSuck(Intake intake) {
      m_intake = intake;
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(intake);
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        //Drop the intake
        if(!m_intake.isDeployed()){
            m_intake.deploy(true);
        }
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        //Run the Intake
        m_intake.feedIn();
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        //Stop the intake
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return m_intake.isDeployed();
    }
}
