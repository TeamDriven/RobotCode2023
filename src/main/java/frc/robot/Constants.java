// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.Elevator;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final double kIntakeSpeed = 0.7;

  public final static class DrivetrainConstants {
    public static final double kPXController = 2.0; // TODO tune this
    public static final double kPYController = 0;//kPXController;
    public static final double kPThetaController = 6; // TODO tune this

    public static final int FLDriveMotorChannel = 3;
    public static final int FRDriveMotorChannel = 5;
    public static final int BLDriveMotorChannel = 1;
    public static final int BRDriveMotorChannel = 7;

    public static final int FLTurningMotorChannel = 4;
    public static final int FRTurningMotorChannel = 6;
    public static final int BLTurningMotorChannel = 2;
    public static final int BRTurningMotorChannel = 8;

    public static final int FLTurningEncoderChannel = 1;
    public static final int FRTurningEncoderChannel = 2;
    public static final int BLTurningEncoderChannel = 0;
    public static final int BRTurningEncoderChannel = 3;

    public static final double FLOffset = 5.649738110618463;
    public static final double FROffset = 0.9418667305466683+3.14;
    public static final double BLOffset = 0.16685627318886792;
    public static final double BROffset = -0.4638748593756933;

    public static final int pigeyDeviceNumber = 11;
  }

  public final static class MotionMagicConstants {
    public static int kPIDLoopIdx = 0;
    public static int kTimeoutMs = 30;
    public static double posOne = 1000;
    public static double posTwo = 100;

    public static double elevatorStartPos = 0;
    public static double elevatorTicksPerInches = 370;
    // public static double elevatorTicksPerInches = 281.9;
    public static double elevator20Inches = 20*elevatorTicksPerInches;
    public static double elevatorUpPos = elevatorTicksPerInches * 55;
    public static double elevatorMidPos = elevatorTicksPerInches * 35;
    public static double elevatorPickUpPos = elevatorTicksPerInches * 17;

    public static double armTicksPerDegree = -318.5778;
    public static double armStartPos = 0;
    public static double armDownPos = armTicksPerDegree * 135;
    public static double armPlacePos = armTicksPerDegree * 30;
    public static double armMaxPos = armTicksPerDegree * 180;
    public static double armBottom = armTicksPerDegree * 100;

    public static final int kSlotIdx = 0;
  }
}
