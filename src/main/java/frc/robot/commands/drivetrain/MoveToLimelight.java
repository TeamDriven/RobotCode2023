// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LimeLight;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class MoveToLimelight extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Drivetrain m_drivetrain;
  private final LimeLight m_limelight; 


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public MoveToLimelight(Drivetrain subsystem, LimeLight limelight) {
    m_drivetrain = subsystem;
    m_limelight = limelight;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.printf("x: %f\r\n",m_limelight.getTX());

    if(m_limelight.getTX()<=-2){
      m_drivetrain.drive(0, 0.02, 0, false);
      
    } else if (m_limelight.getTX()>=2){
      m_drivetrain.drive(0, -0.02, 0, false);
      
    } else {
      m_drivetrain.drive(0, 0, 0, false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if (!(m_limelight.getTX()<=-1 || m_limelight.getTX()>=1)) {
    //   return true;
    // }
    return false;
  }
}
