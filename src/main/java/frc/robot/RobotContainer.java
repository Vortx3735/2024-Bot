// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
// import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArmCom;
import frc.robot.commands.IntakeCom;
import frc.robot.commands.ShooterCom;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.LED;
import frc.robot.util.VorTXControllerXbox;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static VorTXControllerXbox con1 = new VorTXControllerXbox(0);


  public static Intake intake = new Intake(12);
  public static IntakeCom intakecom = new IntakeCom(intake);

  public static Arm arm = new Arm(10, 11, Constants.ArmConstants.motorToArmGearRatio);
  public static ArmCom armcom = new ArmCom(arm);

  public static Shooter shooter = new Shooter(13, 14);
  public static ShooterCom shootercom = new ShooterCom(shooter);
  
  public static LED led = new LED(0, 0);
  

  public static final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve"));


                                                                      



  

                                                                   
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    // AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
    //                                                                () -> MathUtil.applyDeadband(con1.getLeftY(),
    //                                                                                             OperatorConstants.LEFT_Y_DEADBAND),
    //                                                                () -> MathUtil.applyDeadband(con1.getLeftX(),
    //                                                                                             OperatorConstants.LEFT_X_DEADBAND),
    //                                                                () -> MathUtil.applyDeadband(con1.getRightX(),
    //                                                                OperatorConstants.RIGHT_X_DEADBAND),
    //                                                                () -> con1.triangle.getAsBoolean(),
    //                                                                () -> con1.cross.getAsBoolean(),
    //                                                                () -> con1.square.getAsBoolean(),
    //                                                                () -> con1.circle.getAsBoolean(),
    //                                                                18);
    // // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation
    // Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
    //     () -> MathUtil.applyDeadband(con1.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
    //     () -> MathUtil.applyDeadband(con1.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
    //     () -> con1.getRightX(),
    //     () -> con1.getRightY()
    //   );



  //   Shuffleboard.getTab("shooter")
  //  .add("Shooter Setpoint", 2000)
  //  .withWidget(BuiltInWidgets.kNumberSlider) // specify the widget here
  //  .getEntry();
  

    // Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
    //     () -> MathUtil.applyDeadband(con1.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
    //     () -> MathUtil.applyDeadband(con1.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
    //     () -> MathUtil.applyDeadband(con1.getRightX(), OperatorConstants.LEFT_X_DEADBAND),
    //     () -> MathUtil.applyDeadband(con1.getRightY(), OperatorConstants.LEFT_Y_DEADBAND)
    //   );

    Command driveFieldOriented = drivebase.driveCommand(
      () -> MathUtil.applyDeadband(-con1.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
      () -> MathUtil.applyDeadband(-con1.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
      () -> MathUtil.applyDeadband(-con1.getRightX(), OperatorConstants.LEFT_X_DEADBAND)
    );

    // Command driveFieldOrientedSlow = drivebase.driveCommand(
    //   () -> MathUtil.applyDeadband(-con1.getLeftY() / 2, OperatorConstants.LEFT_Y_DEADBAND),
    //   () -> MathUtil.applyDeadband(-con1.getLeftX() / 2, OperatorConstants.LEFT_X_DEADBAND),
    //   () -> MathUtil.applyDeadband(-con1.getRightX() / 2, OperatorConstants.LEFT_X_DEADBAND)
    // );
    
    drivebase.setDefaultCommand(driveFieldOriented);

    arm.setDefaultCommand(
      new RunCommand(
        arm::hold,
        arm  
      )
    );

    if(Intake.hasRing = true){
      shooter.setDefaultCommand(
        new RunCommand(
          () -> shooter.move(.5), 
          shooter)
      );
    } else {
      shooter.setDefaultCommand(
        new RunCommand(
          () -> shooter.move(0),
          shooter
        )
      );
    }


    // // go brrrrrrrrrrrrrrrr and vibrate when we have a ring (hopefully)
    // fIntakesub.setDefaultCommand(
    //   new RunCommand(
    //     () -> {
    //       if (fIntakecom.hasRing(IntakeConstants.ringDistanceMeters, IntakeConstants.ringDistanceErrorMeters)) {
    //         con1.setRumble(RumbleType.kBothRumble, 1);
    //       }
    //     }
    //   )
    // );
    
    intake.setDefaultCommand(
      new RunCommand(
        intake::stopIntake,
        intake
      )
    );
    
    // this is the better one incorporating gravity
    // ethan is bad cole is better
    // all hail king cole
    // https://www.desmos.com/calculator/bvahs1n1ch


    
    
    

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

    con1.menu.whileTrue(
      ( 
        new InstantCommand(
          drivebase::zeroGyro
        )
      )
    );
    
    con1.xButton.whileTrue(
      new SequentialCommandGroup(
        new RunCommand(
          intakecom::intakeNote, 
          intake
          ).until(intake.getBeam()),
          
        new RunCommand(
            intakecom::fixOvershoot, 
            intake
          ).until(intake.getDeadBeam()),

        new RunCommand(
          () -> intake.move(.1), 
          intake).withTimeout(.3),
        
        new InstantCommand(
          intake::ringTrue,
          intake
        )
      )
    );

    con1.bButton.whileTrue(
        new RunCommand(
          () -> intake.move(-.8), 
          intake)
    );


    // con1.l2.whileTrue(
    //   new RunCommand(
    //     () -> shooter.move(.65),
    //     shooter).alongWith(
    //       new RunCommand(
    //         () -> intake.move(.65), 
    //         intake)
    //       )
    //     );

    con1.rb.whileTrue(
      new SequentialCommandGroup(
        new RunCommand(
          () -> shooter.move(1), // rev up shooter
          shooter).withTimeout(2),
        
        new RunCommand(
          () -> intake.move(.65), 
          intake).alongWith(
            new RunCommand(
              () -> shooter.move(1), // rev up shooter
              shooter).withTimeout(1),
            
            new RunCommand(
              () -> intake.move(.65), 
              intake).alongWith(
                new RunCommand(
                  () -> shooter.move(1), // shooter
                  shooter).alongWith(
                    new InstantCommand(
                      intake::ringFalse,
                      intake
                    )
                  )
              )
          )
      )
    );

      con1.yButton.whileTrue(
        new RunCommand(
          () -> shooter.move(.2), 
          shooter).alongWith(
            new RunCommand(
              () -> intake.move(.3), 
              intake)
          )
      );      




    // InstantCommand moveArmToAmp = new InstantCommand(
    //   () -> arm.moveToSetpoint(ArmConstants.ampArmPos, 2),
    //   arm
    // );

        // con1.yButton.whileTrue(
        //   moveArmToAmp
        // );

    //Goes up
    con1.povUp.whileTrue(
      new RunCommand(
        () -> arm.up(0.5),
        arm
      )
    );

    //Goes towards floor
    con1.povDown.whileTrue(
      new RunCommand(
        () -> arm.down(0.5),
        arm
      )
    );

    // con1.r2.whileTrue(
    //   new RunCommand(
    //     armcom::reverseMotor,
    //     armsub
    //     )
    // );

    con1.lb.whileTrue(
      new RunCommand(
        () -> drivebase.driveCommand
        (
          () -> MathUtil.applyDeadband(-con1.getLeftY() / 2, OperatorConstants.LEFT_Y_DEADBAND),
          () -> MathUtil.applyDeadband(-con1.getLeftX() / 2, OperatorConstants.LEFT_X_DEADBAND),
          () -> MathUtil.applyDeadband(-con1.getRightX() / 2, OperatorConstants.LEFT_X_DEADBAND)
        ), drivebase)
    );
    
  }



  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("Test");
  }

  public void setDriveMode() {
    // drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
}
