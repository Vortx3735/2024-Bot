// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.util.VorTXController;
import java.io.File;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  public static VorTXController con1 = new VorTXController(0);


  public static IntakeSub fIntakesub = new IntakeSub(0);
  public static IntakeCOM fIntake = new IntakeCOM(fIntakesub);

  public static ArmSub armsub = new ArmSub(0, 0);
  public static ArmCOM arm = new ArmCOM(armsub);

  public static ShooterSub shootersub = new ShooterSub(0, 0);
  public static ShooterCOM shooter = new ShooterCOM(shootersub);

  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve"));

                                                                   
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
                                                                   () -> MathUtil.applyDeadband(con1.getLeftY(),
                                                                                                OperatorConstants.LEFT_Y_DEADBAND),
                                                                   () -> MathUtil.applyDeadband(con1.getLeftX(),
                                                                                                OperatorConstants.LEFT_X_DEADBAND),
                                                                   () -> MathUtil.applyDeadband(con1.getRightX(),
                                                                   OperatorConstants.RIGHT_X_DEADBAND),
                                                                   () -> con1.triangle.getAsBoolean(),
                                                                   () -> con1.cross.getAsBoolean(),
                                                                   () -> con1.square.getAsBoolean(),
                                                                   () -> con1.circle.getAsBoolean(),
                                                                   18);
    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation
    Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(con1.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(con1.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> con1.getRightX(),
        () -> con1.getRightY());

    drivebase.setDefaultCommand(driveFieldOrientedDirectAngle);

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

        con1.options.onTrue(
          ( 
            new InstantCommand(
              drivebase::zeroGyro
            )
          )
        );

        // intake
        con1.circle.onTrue(
            new InstantCommand(
                fIntake::startMotor,
                fIntakesub
            )                
        );

        con1.square.onTrue(
            new InstantCommand(
                fIntake::stopIntake,
                fIntakesub
            )
        );

        // shooter
        con1.cross.onTrue(
            new InstantCommand(
              shooter::shoot,
              shootersub
            )
        ); 

    }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new InstantCommand();
  }

  public void setDriveMode()
  {
    //drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
