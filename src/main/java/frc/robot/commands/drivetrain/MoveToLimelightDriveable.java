// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import static frc.robot.Controls.*;
import static frc.robot.SubsystemInstances.*;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class MoveToLimelightDriveable extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  double m_heading;
  double currentHeading;
  boolean hasInterruptedTurn;

  private final double m_deadZone = 0.08;

  private PIDController m_turningPIDController = new PIDController(0.01, 0.0, 0.0);

  private PIDController m_YdrivePIDController = new PIDController(0.005, 0.0, 0.0); //0.01
  private PIDController m_XdrivePIDController = new PIDController(0.01, 0.0, 0.0);

  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public MoveToLimelightDriveable(double heading) {
    m_heading = heading;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);

    m_turningPIDController.setTolerance(1);
    m_turningPIDController.enableContinuousInput(-180, 180);
    m_turningPIDController.setSetpoint(m_heading);

    m_YdrivePIDController.setTolerance(1);
    m_YdrivePIDController.setSetpoint(0);
    m_XdrivePIDController.setTolerance(1);
    m_XdrivePIDController.setSetpoint(0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hasInterruptedTurn = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentHeading = drivetrain.getActualHeading();
    double rot = 0;
    hasInterruptedTurn = Math.abs(turnControl.getAsDouble()) > 0.1 || hasInterruptedTurn;

    while (currentHeading > 180) {
      currentHeading -= 360;
    }
    while (currentHeading < -180) {
      currentHeading += 360;
    }

    rot = m_turningPIDController.calculate(currentHeading);

    if (hasInterruptedTurn) {
      rot =
        -m_rotLimiter.calculate(MathUtil.applyDeadband(turnControl.getAsDouble(), m_deadZone))
            * Drivetrain.kMaxAngularSpeed * 0.25;
    } else if (m_turningPIDController.atSetpoint()) {
      rot = 0;
    }

    double ySpeed = m_YdrivePIDController.calculate(-limelight.getTX());
    if (m_YdrivePIDController.atSetpoint()) {
      ySpeed = 0;
    }

    double xSpeed;
    if (Math.abs(xMoveControl.getAsDouble()) > m_deadZone) {
      xSpeed =
        -m_xspeedLimiter.calculate(MathUtil.applyDeadband(xMoveControl.getAsDouble(), m_deadZone))
            * drivetrain.maxSpeed;
    } else {
      xSpeed = m_XdrivePIDController.calculate(-limelight.getTY());
      // System.out.println(-m_limelight.getTY());
      if (m_XdrivePIDController.atSetpoint() || -limelight.getTY() < 0.0) {
        xSpeed = 0;
      }
    }
             
    drivetrain.drive(xSpeed, ySpeed, rot, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if (!(m_limelight.getTX()<=-1 || m_limelight.getTX()>=1)) {
    //   return true;
    // }
    return false;
  }
}
