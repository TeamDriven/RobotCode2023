// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class SetArmPositionWaitForFinish extends CommandBase {
  Arm m_arm; 
  double m_targetPos;

  double tolerance = 250;
  double pauseTime = 0.2;

  double startingTime;
  /** Creates a new setArmPosition. */
  public SetArmPositionWaitForFinish(Arm arm, double targetPos) {
    m_arm = arm;
    m_targetPos = targetPos; 
    addRequirements(arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startingTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_arm.setArmPosition(m_targetPos);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // System.out.println(m_arm.getVelocity());
    if (Math.abs(m_arm.getVelocity()) < tolerance && Timer.getFPGATimestamp() >= startingTime + pauseTime) {
      return true;
    }
    return false;
  }
}
