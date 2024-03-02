// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;;

/**
 * A more advanced Swerve Control System that has 4 buttons for which direction to face
 */
public class AbsoluteDriveAdv extends Command
{

  private final SwerveSubsystem swerve;
  private final DoubleSupplier  vX, vY;
  private final DoubleSupplier  headingAdjust;
  private final BooleanSupplier lookAway, lookTowards, lookLeft, lookRight;
  private       boolean         resetHeading = false;
  Pigeon2         pigeon;

  /**
   * Used to drive a swerve robot in full field-centric mode.  vX and vY supply translation inputs, where x is
   * torwards/away from alliance wall and y is left/right. Heading Adjust changes the current heading after being
   * multipied by a constant. The look booleans are shortcuts to get the robot to face a certian direction. Based off of
   * ideas in https://www.chiefdelphi.com/t/experiments-with-a-swerve-steering-knob/446172
   *
   * @param swerve        The swerve drivebase subsystem.
   * @param vX            DoubleSupplier that supplies the x-translation joystick input.  Should be in the range -1 to 1
   *                      with deadband already accounted for.  Positive X is away from the alliance wall.
   * @param vY            DoubleSupplier that supplies the y-translation joystick input.  Should be in the range -1 to 1
   *                      with deadband already accounted for.  Positive Y is towards the left wall when looking through
   *                      the driver station glass.
   * @param headingAdjust DoubleSupplier that supplies the component of the robot's heading angle that should be
   *                      adjusted. Should range from -1 to 1 with deadband already accounted for.
   * @param lookAway      Face the robot towards the opposing alliance's wall in the same direction the driver is
   *                      facing
   * @param lookTowards   Face the robot towards the driver
   * @param lookLeft      Face the robot left
   * @param lookRight     Face the robot right
   */
  public AbsoluteDriveAdv(SwerveSubsystem swerve, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier headingAdjust,
                          BooleanSupplier lookAway, BooleanSupplier lookTowards, BooleanSupplier lookLeft,
                          BooleanSupplier lookRight, int pigeonID) {
    this.swerve = swerve;
    this.vX = vX;
    this.vY = vY;
    this.headingAdjust = headingAdjust;
    this.lookAway = lookAway;
    this.lookTowards = lookTowards;
    this.lookLeft = lookLeft;
    this.lookRight = lookRight;
    this.pigeon = new Pigeon2(pigeonID);
    this.pigeon.reset();

    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    resetHeading = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double headingX = 0;
    double headingY = 0;

    // These are written to allow combinations for 45 angles
    // Face Away from Drivers
    if (lookAway.getAsBoolean()) {
      headingY = -1;
    }
    // Face Right
    if (lookRight.getAsBoolean()) {
      headingX = 1;
    }
    // Face Left
    if (lookLeft.getAsBoolean()) {
      headingX = -1;
    }
    // Face Towards the Drivers
    if (lookTowards.getAsBoolean()) {
      headingY = 1;
    }

    // Prevent Movement After Auto
    if (resetHeading) {
      if (headingX == 0 && headingY == 0 && Math.abs(headingAdjust.getAsDouble()) > 0) {
        // Get the curret Heading
        Rotation2d currentHeading = pigeon.getRotation2d();

        // Set the Current Heading to the desired Heading
        headingX = currentHeading.getSin();
        headingY = currentHeading.getCos();
      }
      //Dont reset Heading Again
      resetHeading = false;
    }


    ChassisSpeeds desiredSpeeds = swerve.getTargetSpeeds(vX.getAsDouble(), vY.getAsDouble(), headingX, headingY);

    // Limit velocity to prevent tippy
    Translation2d translation = SwerveController.getTranslation2d(desiredSpeeds);
    translation = SwerveMath.limitVelocity(translation, swerve.getFieldVelocity(), swerve.getSwervePose(),
                                           Constants.LOOP_TIME, Constants.ROBOT_MASS, List.of(Constants.CHASSIS),
                                           swerve.getSwerveDriveConfiguration());
    SmartDashboard.putNumber("LimitedTranslation", translation.getX());
    SmartDashboard.putString("Translation", translation.toString());

    // Make the robot move
    if (headingX == 0 && headingY == 0 && Math.abs(headingAdjust.getAsDouble()) > 0) {
      resetHeading = true;
      swerve.drive(translation, (Constants.OperatorConstants.TURN_CONSTANT * -headingAdjust.getAsDouble()), true);
    } else {
      swerve.drive(translation, desiredSpeeds.omegaRadiansPerSecond, true);
    }
  }

  public static Command trackApriltagDrive() {
    //wrap in an if statement for if limelight is reading apriltag
    double displacementX = 0; //getLimelightX
    double goalX = 0; //getLimelight center postition + offset of limelight
    double error = 0; //error for angle
    if(displacementX < goalX - error) {
      return RobotContainer.drivebase.driveCommand(
        () -> MathUtil.applyDeadband(-RobotContainer.con1.getLeftY(), Constants.OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(-RobotContainer.con1.getLeftX(), Constants.OperatorConstants.LEFT_X_DEADBAND),
        () -> .2);
    } else if (displacementX > goalX + error){
      return RobotContainer.drivebase.driveCommand(
        () -> MathUtil.applyDeadband(-RobotContainer.con1.getLeftY(), Constants.OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(-RobotContainer.con1.getLeftX(), Constants.OperatorConstants.LEFT_X_DEADBAND),
        () -> -.2);
    } else {
      return new InstantCommand(); //could make LED blink or smth
    }
  }

  public static Command slowSwerveCommand() {
    return RobotContainer.drivebase.driveCommand(
        () -> MathUtil.applyDeadband(-RobotContainer.con1.getLeftY()*.5, Constants.OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(-RobotContainer.con1.getLeftX()*.5, Constants.OperatorConstants.LEFT_X_DEADBAND),
        () -> MathUtil.applyDeadband(-RobotContainer.con1.getRightX()*.5, Constants.OperatorConstants.LEFT_X_DEADBAND)
    );
  }

  public static Command driveFieldOriented() { 
    return RobotContainer.drivebase.driveCommand(
      () -> MathUtil.applyDeadband(-RobotContainer.con1.getLeftY(), Constants.OperatorConstants.LEFT_Y_DEADBAND),
      () -> MathUtil.applyDeadband(-RobotContainer.con1.getLeftX(), Constants.OperatorConstants.LEFT_X_DEADBAND),
      () -> MathUtil.applyDeadband(-RobotContainer.con1.getRightX(), Constants.OperatorConstants.LEFT_X_DEADBAND)
    );
  }

  public static Command driveDefault() {
    // if(RobotContainer.con1.lb.getAsBoolean() == true) {
    //   return slowSwerveCommand();
    // } else if (RobotContainer.con1.rb.getAsBoolean() == true) {
    //   return trackApriltagDrive();
    // } else {
    //   return driveFieldOriented();
    // }
    return driveFieldOriented();
  }

  public static Command driveDefaultCom() {
      return new RunCommand(
        () -> driveFieldOriented(), 
        RobotContainer.drivebase
      );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }


}