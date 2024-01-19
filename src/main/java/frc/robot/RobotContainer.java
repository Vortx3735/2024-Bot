// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ArmCOM;
import frc.robot.commands.ClimbCOM;
import frc.robot.commands.IntakeCOM;
import frc.robot.commands.ShooterCOM;
import frc.robot.subsystems.ArmSub;
import frc.robot.subsystems.ClimbSub;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSub;
import frc.robot.subsystems.ShooterSub;
import frc.robot.util.VorTXController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  public static VorTXController con1 = new VorTXController(0);
  public static VorTXController con2 = new VorTXController(1);

  public static ClimbSub climbsub = new ClimbSub(0);
  public static ClimbCOM climb = new ClimbCOM(climbsub);

  public static IntakeSub fIntakesub = new IntakeSub(0);
  public static IntakeCOM fIntake = new IntakeCOM(fIntakesub);

  public static ArmSub armsub = new ArmSub(0, 0);
  public static ArmCOM arm = new ArmCOM(armsub);

  public static ShooterSub shootersub = new ShooterSub(0, 0);
  public static ShooterCOM shooter = new ShooterCOM(shootersub);

  public static DriveSubsystem swerve = new DriveSubsystem();
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    swerve.setDefaultCommand(
            // The left stick controls translation of the robot.
            // Turning is controlled by the X axis of the right stick.
            new RunCommand(
                () -> swerve.drive(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                        //1,0,0,
                        MathUtil.applyDeadband(con1.getLeftY(), 0.1),
                        MathUtil.applyDeadband(con1.getLeftX(), 0.1),
                        MathUtil.applyDeadband(con1.getRightX(), 0.1)*1.75, 
                        DriveSubsystem.getGyroscopeRotation())
                    ),
                    swerve
            )
        );
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
    // R1 changes speed to as fast as possible
        // L1 changes speed to really slow
        // if speed is set to anything > 4, motor controllers set to brake 

        // more swerve
        con1.r2.onTrue(
            new InstantCommand(
                () -> {
                    swerve.changeSpeed(10);
                },
                swerve
            )
        );

        //swerve
        con1.l2.onTrue(
            new InstantCommand(
                () -> {
                    swerve.changeSpeed(1.5);
                },
                swerve
            )
        );

        // share button on left of ps4 controller
        con1.share.onTrue(
           new InstantCommand(
                swerve::zeroGyroscope,
                swerve
           ) 
        );

        // climb
        con1.triangle.whileTrue(
             new RunCommand(
                 climb::reverseMotor,
                 climbsub
             )
         );

        // intake
        con1.circle.onTrue(
            new InstantCommand(
                fIntake::push,
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
}
