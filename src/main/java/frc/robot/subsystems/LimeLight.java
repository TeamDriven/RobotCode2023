// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimeLight extends SubsystemBase {
  public static final String LIMELIGHT = "limelight";
  public static double tx = 0;

  /** Creates a new ExampleSubsystem. */
  public LimeLight() {
    updateLimeLight();

    turnOffLimelight();
    setLights(1);
  }

  public void turnOnLimelight() {
    NetworkTableInstance.getDefault().getTable(LIMELIGHT).getEntry("camMode").setNumber(0);
  }

  public void turnOffLimelight() {
    NetworkTableInstance.getDefault().getTable(LIMELIGHT).getEntry("camMode").setNumber(1);
  }

  // 0 for default, 1 for off, 2 for blink, 3 for on
  public void setLights(int lightMode) {
    NetworkTableInstance.getDefault().getTable(LIMELIGHT).getEntry("ledMode").setNumber(lightMode);
  } 

  // 0 for Apriltags, 1 for Retroreflective Tape
  public void setLimelightPipeline(int pipelineNumber) {
    NetworkTableInstance.getDefault().getTable(LIMELIGHT).getEntry("pipeline").setNumber(pipelineNumber);
  }

  public void updateLimeLight() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable(LIMELIGHT);
    tx = table.getEntry("tx").getDouble(0);
    double targetOffsetAngle_Vertical = table.getEntry("ty").getDouble(0);
    double targetArea = table.getEntry("ta").getDouble(0);
    double targetSkew = table.getEntry("ts").getDouble(0);

    // table.putValue("ledMode", 3);

    // final ShuffleboardTab tab = Shuffleboard.getTab(LIMELIGHT);
    SmartDashboard.putNumber("ta", targetArea);
    SmartDashboard.putNumber("ts", targetSkew);
    SmartDashboard.putNumber("tx", tx);
    SmartDashboard.putNumber("ty", targetOffsetAngle_Vertical);
  }

  public void resetLimelight() {
    tx = 0;
    double targetOffsetAngle_Vertical = 0;
    double targetArea = 0;
    double targetSkew = 0;
    SmartDashboard.putNumber("ta", targetArea);
    SmartDashboard.putNumber("ts", targetSkew);
    SmartDashboard.putNumber("tx", tx);
    SmartDashboard.putNumber("ty", targetOffsetAngle_Vertical);

    setLights(1);
    turnOffLimelight();
  }

  public void readAprilTags() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable(LIMELIGHT);
    
  }

  public double getX() {
    return tx;
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}