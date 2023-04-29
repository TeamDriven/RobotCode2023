// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import static frc.robot.Controls.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class AutoTurnToSubstation extends CommandBase {
  /** Creates a new AutoBalance. */
  Drivetrain m_drivetrain;
  // double m_heading;
  double currentHeading;
  // double leeway = 1;

  private final double m_deadZone = 0.08;

  private PIDController m_turningPIDController = new PIDController(0.01, 0.0, 0.0);

  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  
  public AutoTurnToSubstation(Drivetrain drivetrain) {
    m_drivetrain = drivetrain;
    addRequirements(m_drivetrain);

    m_turningPIDController.setTolerance(1);
    m_turningPIDController.enableContinuousInput(-180, 180);
  }

  private double turnToAlliance() {
    // System.out.println("alliance: " + DriverStation.getAlliance().name() + " Blue: " + Alliance.Blue.name() + " Red: " + Alliance.Red.name());
    if (DriverStation.getAlliance().name().equals(Alliance.Blue.name())) {
      return 90;
    } else if (DriverStation.getAlliance().name().equals(Alliance.Red.name())) {
      return -90;
    } else {
      return 1000;
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double m_heading = turnToAlliance();
    m_turningPIDController.setSetpoint(m_heading);
    currentHeading = m_drivetrain.getActualHeading();
    double rot = 0;

    // double speed = 0.1;

    // if (negativeTurnLimit < -180) {
    //   positiveTurnLimit = negativeTurnLimit + 360;
    //   negativeTurnLimit = m_heading;
    //   speed = -speed;
    // }
    
    final var xSpeed =
        -m_xspeedLimiter.calculate(MathUtil.applyDeadband(xMoveControl.getAsDouble(), m_deadZone))
            * m_drivetrain.maxSpeed;

    final var ySpeed =
        -m_yspeedLimiter.calculate(MathUtil.applyDeadband(yMoveControl.getAsDouble(), m_deadZone))
            * m_drivetrain.maxSpeed;

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

    // if (Math.abs(m_heading - currentHeading) > leeway) {
    //   if (currentHeading >= positiveTurnLimit || currentHeading <= negativeTurnLimit) {
    //     rot = -speed;
    //   } else {
    //     rot = speed;
    //   }
    // }

    m_drivetrain.drive(xSpeed, ySpeed, rot, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.drive(0, 0, 0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if (m_turningPIDController.atSetpoint()) {
    //   return true;
    // }
    return false;
  }
}


