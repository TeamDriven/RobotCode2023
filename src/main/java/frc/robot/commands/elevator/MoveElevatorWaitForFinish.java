// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.Timer;
// import static frc.robot.Constants.MotionMagicConstants.*;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

/** An example command that uses an example subsystem. */
public class MoveElevatorWaitForFinish extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Elevator m_elevator;
  private final double m_targetPos;

  double tolerance = 50;
  double pauseTime = 0.2;

  double startingTime;


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public MoveElevatorWaitForFinish(Elevator subsystem, double targetPos) {
    m_elevator = subsystem;
    m_targetPos = targetPos;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startingTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // double targetPos = m_targetPos;
    // if (m_elevator.targetPos == targetPos) {
      //targetPos = elevatorStartPos;
    // }
    m_elevator.motionMagicElevator(m_targetPos);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //System.out.println(m_elevator.getVelocity());
    if (Math.abs(m_elevator.getVelocity()) < tolerance && Timer.getFPGATimestamp() >= startingTime + pauseTime) {
      return true;
    }
    return false;
  }
}
