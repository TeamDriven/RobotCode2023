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
public class DriveContinousPosition extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private PIDController m_turningPIDController = new PIDController(0.01, 0.0, 0.0);
  private double m_heading = 180;

  private final double m_deadZone = 0.08;

  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  // private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveContinousPosition() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);

    m_turningPIDController.setTolerance(1);
    m_turningPIDController.enableContinuousInput(-180, 180);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    final var xSpeed =
        -m_xspeedLimiter.calculate(MathUtil.applyDeadband(xMoveControl.getAsDouble(), m_deadZone))
            * drivetrain.maxSpeed;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    final var ySpeed =
        -m_yspeedLimiter.calculate(MathUtil.applyDeadband(yMoveControl.getAsDouble(), m_deadZone))
            * drivetrain.maxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    // final var rot =
    //     -m_rotLimiter.calculate(MathUtil.applyDeadband(turnControl.getAsDouble(), m_deadZone))
    //         * Drivetrain.kMaxAngularSpeed * 0.25;

    final var rotX = 
          MathUtil.applyDeadband(m_controller.getLeftX(), m_deadZone);
    final var rotY = 
          MathUtil.applyDeadband(m_controller.getLeftY(), m_deadZone);

    double currentHeading = drivetrain.getActualHeading();
    while (currentHeading > 180) {
      currentHeading -= 360;
    }
    while (currentHeading < -180) {
      currentHeading += 360;
    }

    if (Math.abs(rotX) >= m_deadZone || Math.abs(rotY) >= m_deadZone){
      m_heading = Math.toDegrees(Math.atan2(rotY, rotX)) + 90;
      while (m_heading > 180) {
        m_heading -= 360;
      }

      m_heading *= -1;
    } else {
      m_heading = currentHeading;
    }
    // while (m_heading < -180) {
    //   m_heading += 360;
    // }
    

    System.out.println(rotX);
    System.out.println(rotY);
    System.out.println(m_heading);

    m_turningPIDController.setSetpoint(m_heading);

    double rot = m_turningPIDController.calculate(currentHeading);
    if (m_turningPIDController.atSetpoint()) {
      rot = 0;
    }
          

    drivetrain.drive(xSpeed, ySpeed, rot, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
