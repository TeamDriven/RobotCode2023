// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ClawPneumatics;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetClawPnuematics extends InstantCommand {
  ClawPneumatics m_claw;
  boolean m_openClose;
  public SetClawPnuematics(ClawPneumatics clawPneumatics, boolean openClose) {
    addRequirements(clawPneumatics);
    m_claw = clawPneumatics;
    m_openClose = openClose;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_claw.setGrip(m_openClose);
  }
}
