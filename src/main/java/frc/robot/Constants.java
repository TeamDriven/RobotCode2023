// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;

// import frc.robot.subsystems.Elevator;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final double kIntakeSpeed = 1;

  public final static class DrivetrainConstants {
    public static final double kSlowDriveSpeed = 1;
    public static final double kFastDriveSpeed = 6;

    public static final PIDController kAutoDrivePID = new PIDController(0.13433, 0, 0);
    public static final PIDController kTeleOpDrivePID = new PIDController(0.75, 0, 0);

    public static final double kPXController = 0.7;
    public static final double kPYController = 0.7;
    public static final double kPThetaController = 3;

    public static final double rollTarget = 11;
  }

  public final static class MotionMagicConstants {
    public static int kPIDLoopIdx = 0;
    public static int kTimeoutMs = 30;
    public static double posOne = 1000;
    public static double posTwo = 100;

    public static double elevatorStartPos = 0;
    public static double elevatorTicksPerInches = 370;
    public static double elevator20Inches = 20*elevatorTicksPerInches;
    public static double elevatorUpPosAuto = elevatorTicksPerInches * 52;

    public static double elevatorConeUpPos = elevatorTicksPerInches * 54;
    public static double elevatorConeMidPos = elevatorTicksPerInches * 28;
    public static double elevatorPickUpConePos = elevatorTicksPerInches * 10;

    public static double elevatorCubeUpPos = elevatorTicksPerInches * 50;
    public static double elevatorCubeMidPos = elevatorTicksPerInches * 25;
    public static double elevatorPickUpCubePos = elevatorTicksPerInches * 13;

    public static double elevatorTuckPos = elevatorTicksPerInches * 3;

    public static double elevatorSubstationPos = elevatorTicksPerInches * 2;
    public static double elevatorAutoConeUpPos = elevatorTicksPerInches * 41;
    public static double elevatorAutoConeMidPos = elevatorTicksPerInches * 27;

    public static double armTicksPerDegree = 955.73;
    // public static double armTicksPerDegree = 1137.778; // practice bot
    // public static double armTicksPerDegree = 716.8;
    public static double armStartPos = 0;
    public static double armTuckPos = armTicksPerDegree * 10;
    public static double armUpPos = armTicksPerDegree * 13;

    public static double armConePickupPos = armTicksPerDegree * 82;
    public static double armHighPlaceConePos = armTicksPerDegree * 75; //41
    public static double armMidPlaceConePos = armTicksPerDegree * 70; //36

    public static double armCubePickupPos = armTicksPerDegree * 80;
    public static double armHighPlaceCubePos = armTicksPerDegree * 15;
    public static double armMidPlaceCubePos = armTicksPerDegree * 35;

    public static double armSubstationPos = armTicksPerDegree * 30;
    public static double armHighPlaceAutoConePos = armTicksPerDegree * 35;
    public static double armMidPlaceAutoConePos = armTicksPerDegree * 45;

    public static double armOnHighPole = armTicksPerDegree * 30;
    public static double armOnMidPole = armTicksPerDegree * 30;

    public static double armPlacePosAuto = armTicksPerDegree * 65;
    public static double armMaxPos = armTicksPerDegree * 180;
    public static double armBottom = armTicksPerDegree * 100;

    public static final int kSlotIdx = 0;
  }
}
