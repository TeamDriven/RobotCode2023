// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import static frc.robot.Constants.DrivetrainConstants.*;

public class AutoBalance extends CommandBase {
  /** Creates a new AutoBalance. */
  Drivetrain m_drivetrain;
  double speed = 0.3;
  public AutoBalance(Drivetrain drivetrain) {
    m_drivetrain = drivetrain;
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Roll: " + m_drivetrain.getRoll());
    System.out.println("Yaw: " + m_drivetrain.getYaw());
    if (m_drivetrain.getYaw() > 90 || m_drivetrain.getYaw() < -90) {
      if (m_drivetrain.getRoll() > rollTarget){
        m_drivetrain.drive(-speed, 0, 0, false);
      } else if (m_drivetrain.getRoll() < -rollTarget) {
        m_drivetrain.drive(speed, 0, 0, false);
      } else {
        m_drivetrain.boxWheels();
      }
    // } else if (m_drivetrain.getYaw() > 45) {
    //   if (m_drivetrain.getPitch() > rollTarget){
    //     m_drivetrain.drive(-speed, 0, 0, false);
    //   } else if (m_drivetrain.getPitch() < -rollTarget) {
    //     m_drivetrain.drive(speed, 0, 0, false);
    //   } else {
    //     m_drivetrain.boxWheels();
    //   }
    // } else if (m_drivetrain.getYaw() < -45) {
    //   if (m_drivetrain.getPitch() > rollTarget){
    //     m_drivetrain.drive(speed, 0, 0, false);
    //   } else if (m_drivetrain.getPitch() < -rollTarget) {
    //     m_drivetrain.drive(-speed, 0, 0, false);
    //   } else {
    //     m_drivetrain.boxWheels();
    //   }
    } else {
      if (m_drivetrain.getRoll() > rollTarget){
        m_drivetrain.drive(-speed, 0, 0, false);
      } else if (m_drivetrain.getRoll() < -rollTarget) {
        m_drivetrain.drive(speed, 0, 0, false);
      } else {
        m_drivetrain.boxWheels();
      }
    }


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // m_drivetrain.drive(0, 0, 0, true);
    m_drivetrain.boxWheels();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if (m_drivetrain.getRoll() <= rollTarget && m_drivetrain.getRoll() >= -rollTarget){
    //   return true;
    // }
    return false;
  }
}
