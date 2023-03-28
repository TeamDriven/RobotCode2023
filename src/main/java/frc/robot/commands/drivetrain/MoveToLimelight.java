// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LimeLight;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class MoveToLimelight extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Drivetrain m_drivetrain;
  private final LimeLight m_limelight; 

  private PIDController m_drivePIDController = new PIDController(0.01, 0.0, 0.0);


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

    m_drivePIDController.setTolerance(1);
    m_drivePIDController.setSetpoint(0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double ySpeed = m_drivePIDController.calculate(m_limelight.getTX());
    if (m_drivePIDController.atSetpoint()) {
      ySpeed = 0;
    }
             
    m_drivetrain.drive(0.0, ySpeed, 0.0, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //System.out.println(m_limelight.getTX());
    if (m_drivePIDController.atSetpoint()) {
      return true;
    }
    return false;
  }
}
