// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LimeLight;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class EstimatePose extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Drivetrain m_drivetrain;
  private final LimeLight m_limelight; 

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public EstimatePose(Drivetrain subsystem, LimeLight limelight) {
    m_drivetrain = subsystem;
    m_limelight = limelight;
    // Use addRequirements() here to declare subsystem dependencies.
    // addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double[] position = m_limelight.getRobotPose();
    // for (double h : position) {
    //   System.out.print(":" + h + ":\n");
    // }
    Pose2d pose = new Pose2d(position[0], position[1], Rotation2d.fromDegrees(position[5]));
    // System.out.println("X: " + pose.getX() + " Y: " + pose.getY() + " Rot: " + pose.getRotation());
    if (position[0] == 0 && position[1] == 0 && position[3] == 0) {}
    else {
      m_drivetrain.resetOdometry(pose);
    }
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
