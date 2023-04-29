// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class AutoResetArmPosition extends CommandBase {
  Arm m_arm; 
  double timeOut = 1.0;
  double startingTime;
  public AutoResetArmPosition(Arm arm) {
    m_arm = arm;
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
    m_arm.runMotor(-0.2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.stopMotor();
    m_arm.zeroPosition();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_arm.isLimitSwitchPressed() || Timer.getFPGATimestamp() >= startingTime + timeOut) {
      return true;
    } else {
      return false;
    }
  }
}
