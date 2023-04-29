// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import static frc.robot.Controls.*;
import static frc.robot.SubsystemInstances.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoTurn extends CommandBase {
  /** Creates a new AutoBalance. */
  double m_heading;
  double currentHeading;
  // double leeway = 10;

  private PIDController m_turningPIDController = new PIDController(0.01, 0.0, 0.0);

  private final double m_deadZone = 0.08;

  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  
  public AutoTurn(double heading) {
    m_heading = heading;
    addRequirements(drivetrain);
    
    m_turningPIDController.setTolerance(1);
    m_turningPIDController.enableContinuousInput(-180, 180);
    m_turningPIDController.setSetpoint(m_heading);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentHeading = drivetrain.getActualHeading();
    double rot = 0;
    
    // double targetHeading = m_heading;
    // double speed = 0.01;

    // double positiveTurnLimit = m_heading;
    // double negativeTurnLimit = m_heading - 180;

    // if (negativeTurnLimit < -180) {
    //   positiveTurnLimit = negativeTurnLimit + 360;
    //   negativeTurnLimit = m_heading;
    //   speed = -speed;
    // }
    
    final var xSpeed =
        -m_xspeedLimiter.calculate(MathUtil.applyDeadband(xMoveControl.getAsDouble(), m_deadZone))
            * drivetrain.maxSpeed;

    final var ySpeed =
        -m_yspeedLimiter.calculate(MathUtil.applyDeadband(yMoveControl.getAsDouble(), m_deadZone))
            * drivetrain.maxSpeed;
    
    // currentHeading = (currentHeading % 360) - 180;

    while (currentHeading > 180) {
      currentHeading -= 360;
    }
    while (currentHeading < -180) {
      currentHeading += 360;
    }

    rot = m_turningPIDController.calculate(currentHeading);
    if (m_turningPIDController.atSetpoint()) {
      rot = 0;
    }
    // System.out.println("Rot: " + rot + " atSetpoint: " + m_turningPIDController.atSetpoint());

    drivetrain.drive(xSpeed, ySpeed, rot, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(0, 0, 0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // System.out.println("target: " + m_heading + " current: " + currentHeading);
    // if (m_turningPIDController.atSetpoint()) {
    //   return true;
    // }
    return false;
  }
}


