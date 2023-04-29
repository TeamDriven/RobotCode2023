// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import static frc.robot.SubsystemInstances.*;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoResetElevatorPosition extends CommandBase {
  double timeOut = 1.0;
  double startingTime;
  /** Creates a new setArmPosition. */
  public AutoResetElevatorPosition() {
    addRequirements(elevator);
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
    elevator.runElevator(-0.3);
    // System.out.println("Elevator: " + m_elevator.isLimitSwitchPressed());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.runElevator(0);
    elevator.zeroPosition();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (elevator.isLimitSwitchPressed() || Timer.getFPGATimestamp() >= startingTime + timeOut) {
      return true;
    } else {
      return false;
    }
  }
}
