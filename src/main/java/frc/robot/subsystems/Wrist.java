// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase {
  /** Creates a new Wrist. */
  private WPI_TalonFx wristMotor;
  
  public Wrist() {
    wristMotor = new WPI_TalonFx(Constants.WRISTMOTOR_ID);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setwristMotor(double d) {
    wristMotor.set(ControlMode.PercentOutput, d);
  }

  public RunWrist(wrist, wrist boolean inwards) {
    m_wrist = wrist;
    m_inwards = inwards;

    addRequirments(wrist);
  }
}


