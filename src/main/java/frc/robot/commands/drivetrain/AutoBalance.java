// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import static frc.robot.SubsystemInstances.*;

import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.Constants.DrivetrainConstants.*;

public class AutoBalance extends CommandBase {
  /** Creates a new AutoBalance. */
  double speed = 0.2; //0.2
  public AutoBalance() {
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //System.out.println("Roll: " + drivetrain.getRoll());
    //System.out.println("Yaw: " + drivetrain.getYaw());
    if (drivetrain.getYaw() > 90 || drivetrain.getYaw() < -90) {
      if (drivetrain.getRoll() > rollTarget){
        drivetrain.drive(-speed, 0, 0, false);
      } else if (drivetrain.getRoll() < -rollTarget) {
        drivetrain.drive(speed, 0, 0, false);
      } else {
        drivetrain.boxWheels();
      }
    // } else if (drivetrain.getYaw() > 45) {
    //   if (drivetrain.getPitch() > rollTarget){
    //     drivetrain.drive(-speed, 0, 0, false);
    //   } else if (drivetrain.getPitch() < -rollTarget) {
    //     drivetrain.drive(speed, 0, 0, false);
    //   } else {
    //     drivetrain.boxWheels();
    //   }
    // } else if (drivetrain.getYaw() < -45) {
    //   if (drivetrain.getPitch() > rollTarget){
    //     drivetrain.drive(speed, 0, 0, false);
    //   } else if (drivetrain.getPitch() < -rollTarget) {
    //     drivetrain.drive(-speed, 0, 0, false);
    //   } else {
    //     drivetrain.boxWheels();
    //   }
    } else {
      if (drivetrain.getRoll() > rollTarget){
        drivetrain.drive(-speed, 0, 0, false);
      } else if (drivetrain.getRoll() < -rollTarget) {
        drivetrain.drive(speed, 0, 0, false);
      } else {
        drivetrain.boxWheels();
      }
    }


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // drivetrain.drive(0, 0, 0, true);
    drivetrain.boxWheels();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if (drivetrain.getRoll() <= rollTarget && drivetrain.getRoll() >= -rollTarget){
    //   return true;
    // }
    return false;
  }
}
