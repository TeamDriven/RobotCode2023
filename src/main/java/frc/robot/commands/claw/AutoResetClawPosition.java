// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.claw;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class AutoResetClawPosition extends CommandBase {
  Claw m_claw; 
  double timeOut = 1.0;
  double startingTime;
  /** Creates a new setClawPosition. */
  public AutoResetClawPosition(Claw claw) {
    m_claw = claw;
    addRequirements(claw);
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
    m_claw.runMotor(-0.2);
    // System.out.println("Claw: " + m_claw.isLimitSwitchPressed());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_claw.stopMotor();
    m_claw.zeroPosition();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_claw.isLimitSwitchPressed() || Timer.getFPGATimestamp() >= startingTime + timeOut) {
      return true;
    } else {
      return false;
    }
  }
}
