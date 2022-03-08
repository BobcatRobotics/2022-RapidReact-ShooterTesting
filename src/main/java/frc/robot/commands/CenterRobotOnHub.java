// package frc.robot.commands;

// import edu.wpi.first.wpilibj.Joystick;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.Drivetrain;
// import frc.robot.Constants.RobotContainerConstants;
// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableInstance;

// public class CenterRobotOnHub extends CommandBase {

//     private final Drivetrain m_drivetrain;
//     private Joystick m_rightStick;
//     private NetworkTable limelightTable;
    
//     public CenterRobotOnHub(Drivetrain drivetrain, Joystick rightStick) {
//         m_drivetrain = drivetrain;
//         m_rightStick = rightStick;
//         limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
//         addRequirements(drivetrain);
//     }

//     // Called when the command is initially scheduled.
//     @Override
//     public void initialize() {}

//     // Called every time the scheduler runs while the command is scheduled.
//     @Override
//     public void execute() {
//         // Run command when top button on joystick is pressed
//         if (m_rightStick.getTop()) {
//             // Limelight has not found any targets
//             if (!limelightTable.getEntry("tv").getBoolean(false)) {
//                 // Stop drivetrain
//                 m_drivetrain.stop();
//             }
//             // Limelight has found a target
//             else {
//                 // Get horizontal offset from crosshair to target
//                 double tx = limelightTable.getEntry("tx").getDouble(0.0);
//                 // Normalize between 0 and 1 by dividing by max degree offset: 29.8 deg
//                 tx /= 29.8;
//                 // Turn
//                 if (Math.abs(tx) < 0.2) { // Threshold to stop turning
//                     m_drivetrain.stop();
//                 } else {
//                     // Give at least a little bit of turn just in case
//                     if (tx > 0) tx += 0.07;
//                     else tx -= 0.07;
//                     // Make sure not over 1 or below -1
//                     if (tx > 1) tx = 1;
//                     else if (tx < -1) tx = -1;
//                     // Turn
//                     m_drivetrain.drive(tx, -tx);
//                 }
//             }
//         }
//     }

//     // Called once the command ends or is interrupted.
//     @Override
//     public void end(boolean interrupted) {}

//     // Returns true when the command should end.
//     @Override
//     public boolean isFinished() {
//         return false;
//         // return limelightTable.getEntry("tv") && Math.abs(limelightTable.getEntry("tx").getDouble(defaultValue));
//     }

// }
