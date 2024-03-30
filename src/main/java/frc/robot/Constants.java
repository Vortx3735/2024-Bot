// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.util.LinearInterpolator;
import swervelib.math.Matter;
import swervelib.parser.PIDFConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {


  //meesheeshooballooogaggoogaggo woogo wogo bo
  //everything is in metric units and degrees
  
  public static final double MASS_IN_POUNDS = (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) ? 116.5+13.5 : 116.5+11.2;
  public static final double ROBOT_MASS = MASS_IN_POUNDS*0.453592; //converting pounds to kg
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag

  public static final class Auton {
    public static final PIDFConfig TranslationPID = new PIDFConfig(0.7, 0, 0);
    public static final PIDFConfig angleAutoPID   = new PIDFConfig(0.4, 0, 0.01);

    public static final double MAX_ACCELERATION = 2;
  }

  public static final class Drivebase {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
    public static final double maximumSpeed = Units.feetToMeters(17.1);
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 1;
    // Joystick Deadband
    public static final double LEFT_X_DEADBAND  = 0.1;
    public static final double LEFT_Y_DEADBAND  = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double RIGHT_Y_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }

  public static class IntakeConstants {
    public static final double ringDistanceMeters = 0.3;
    public static final double ringDistanceErrorMeters = 0.05;
    public static final int BEAM_BREAK_PORT = 1;
  }


  public static class ArmConstants {
    public static final double motorToArmGearRatio = 1.0/27.0;
    public static final double ampArmPos = 0.570;
    public static final double groundArmPos = 0.314;
    public static final double armLength = Units.inchesToMeters(24.727914);
    public static final double encoderPitchDiameter = 62.23*1000;
    public static final double pivotHeight = Units.inchesToMeters(10.907);
    public static final double[][] armAngleArray = {
      // test for different angles from different x distances
      {0, 0}
    };
    public static final LinearInterpolator interpolateArmAngle = new LinearInterpolator(armAngleArray);
  }

  public static class ShooterConstants {
    public static final double shooterRadius = Units.inchesToMeters(2.0);
    public static final double shooterLength = Units.inchesToMeters(5.639686);
    public static final double maxRPM = 5700;
    public static final double differenceFromArm = Units.degreesToRadians(55);
    public static final double[][] shooterSpeedArray = {
      // test for different speeds from different angles
      {0, 0}
    };
    public static final LinearInterpolator interpolateShooterSpeed = new LinearInterpolator(shooterSpeedArray);

  }

  public static class VisionConstants {
    public static final double limelightHeight = Units.inchesToMeters(15.325);
    public static final double limelightDegrees = 50;
    public static final double limelightXDistToArmPivot = Units.inchesToMeters(2.142);
    public static final double limelightYDistToArmPivot = Units.inchesToMeters(4.418);
    public static final int speakerMidTag = (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) ? 4 : 7;
    public static final int speakerSideTag = (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) ? 3 : 8;
    public static final int ampTags = (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) ? 5 : 6;
  }

  public static class FieldConstants {
    public static final double speakerHeight = 2.045;
    public static final double ampHeight = 0.0;
  }

  public static class ClimbConstants {
    public static final double CLIMB_LENGTH = 0.9;
    public static final float CLIMB_SOFT_LIMIT_FWD = 0;
  }


}