// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.MotionMagicConstants.*;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

/** An example command that uses an example subsystem. */
public class MoveElevatorManual extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Elevator m_elevator;
  private final double m_speed;


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public MoveElevatorManual(Elevator subsystem, double speed) {
    m_elevator = subsystem;
    m_speed = speed;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double targetPos = m_elevator.targetPos + m_speed;
    if (targetPos > elevatorUpPos) {
      targetPos = elevatorUpPos;
    } else if (targetPos < elevatorStartPos) {
      targetPos = elevatorStartPos;
    }
    System.out.println(targetPos);
    m_elevator.motionMagicElevator(targetPos);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
