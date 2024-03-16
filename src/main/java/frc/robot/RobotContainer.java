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
import frc.robot.commands.AutoAim;
import frc.robot.commands.ClimbCom;
import frc.robot.commands.IntakeCom;
import frc.robot.commands.ShooterCom;
import frc.robot.subsystems.AdvantageScope;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Climb;
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
  public static VorTXControllerXbox con2 = new VorTXControllerXbox(1);

  public static Intake intake = new Intake(16);
  public static IntakeCom intakecom = new IntakeCom(intake);

  public static Arm arm = new Arm(12, 13, Constants.ArmConstants.motorToArmGearRatio);
  public static ArmCom armcom = new ArmCom(arm);

  public static Shooter shooter = new Shooter(15, 14);
  public static ShooterCom shootercom = new ShooterCom(shooter);
  
  public static AutoAim autoaim = new AutoAim(arm, shooter);
  
  // public static LED led = new LED(1, 36);

  public static Climb climb = new Climb(10, 11);
  public static ClimbCom climbcom = new ClimbCom(climb);
  

  public static final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve"));


  public static AdvantageScope advantagescope = new AdvantageScope();

                                                                   
  
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
    
    drivebase.setDefaultCommand(driveFieldOriented);

    arm.setDefaultCommand(
      new RunCommand(
        arm::hold,
        arm  
      )
    );

    shooter.setDefaultCommand(shootercom.shooterIdle());


    // climb.setDefaultCommand(
    //   RobotContainer.climbcom.getDefaultCommand()
    // );


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

    climb.setDefaultCommand(
      new RunCommand(
        () -> climb.coast(),
        climb)
    );
    
    // desmos from ayman that calculates arm angle, and scales rpm along with how far we get
    // cole is unsure of the speaker measurements in this
    // https://www.desmos.com/calculator/eqe7eypeiw

    // desmos from cole that calculates arm angle and calculates rpm
    // not calibrated bc we need to know the distance between the arm pivot and the subwoofer,
    // and we need to know the starting angle of the arm when against the subwoofer
    // https://www.desmos.com/calculator/jy7vomzel0
    

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
      new InstantCommand(
        drivebase::zeroGyro
      )
    );
    
    con2.xButton.whileTrue(
      new SequentialCommandGroup(
        intakecom.intakeNoteCom(),
        
        new InstantCommand(
          intake::ringTrue,
          intake
        )
      )
    );

    if(intake.getRing()) {
      con1.rb.whileFalse(
        new RunCommand(
          () -> shooter.move(.5), 
          shooter)
      );
    }

    con2.bButton.whileTrue(
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

    con2.rb.whileTrue(
      new SequentialCommandGroup(
        new RunCommand(
          () -> shooter.move(.8), // rev up shooter
          shooter).withTimeout(2),
        
        new RunCommand(
          () -> intake.move(.5), 
          intake).alongWith(
        new RunCommand(
          () -> shooter.move(.8), // rev up shooter
          shooter).withTimeout(1.25)
            
            // new RunCommand(
            //   () -> intake.move(.65), 
            //   intake).alongWith(
            //     new RunCommand(
            //       () -> shooter.move(1), // shooter
            //       shooter)
            //   )
            
          ),
          new InstantCommand(
              () -> intake.ringFalse(),
              intake
          )
      )
    );

      con2.yButton.whileTrue(
        new RunCommand(
          () -> shooter.move(.5), 
          shooter).alongWith(
            new RunCommand(
              () -> intake.move(.5), 
              intake)
          )
      );      




    RunCommand moveArmToAmp = new RunCommand(
      () -> arm.moveToSetpoint(Constants.ArmConstants.ampArmPos, 7),
      arm
    );

    con2.povLeft.whileTrue(
      moveArmToAmp
    );

    //Goes up
    con1.povLeft.whileTrue(
      new RunCommand(
        () -> arm.up(0.5),
        arm
      )
    );
    
    RunCommand moveArmToGround = new RunCommand(
      () -> arm.moveToSetpoint(Constants.ArmConstants.groundArmPos, 4),
      arm
    );

    //Goes towards floor
    con2.povRight.whileTrue(
      moveArmToGround
    );


    con1.povRight.whileTrue(
      new RunCommand(
        () -> arm.down(.5),
        arm
        )
    );

    con1.lb.onTrue(
      new InstantCommand(
        () -> drivebase.togglePrecisionMode(),
        drivebase)
      );

    con1.rb.onTrue(
      new InstantCommand(
        () -> drivebase.toggleTrackSpeakerMode(),
      drivebase)
    );

    // con2.aButton.whileTrue(
    //   RobotContainer.climbcom.getMoveCommand(-.25)
    // );

    con2.povDown.whileTrue(
      new RunCommand(
        () -> climb.moveRight(-.25), 
        climb)
    );

    // // con2.yButton.whileTrue(climbcom.getMoveCommand(.25));
    // con1.povUp.whileTrue(
    //   new RunCommand(
    //     () -> climb.moveLeft(.25), 
    //     climb)
    // );

    // con1.povUp.whileTrue(
    //   new RunCommand(
    //     () -> climb.moveLeft(.25), 
    //     climb)
    // );

    // con2.view.whileTrue(
    //   new RunCommand(
    //     // () -> climb.setReverseSoftLimit(-5000), 
    //     climb)
    // );

    con1.lt.whileTrue(
      new RunCommand(
        () -> climb.moveBoth(-0.5),
        climb)
    );

    con1.rt.whileTrue(
      new RunCommand(
        () -> climb.moveBoth(0.5),
        climb)
    );

  }



  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    
    // TEST, center 3 note auton (including pre-loaded note)
    // return drivebase.getAutonomousCommand("Test");

    // StartWLeft, 3 note auton (including pre-loaded note)
    // return drivebase.getAutonomousCommand("StartWLeft");

    // StartWRight, 3 note auton (including pre-loaded note)
    // return drivebase.getAutonomousCommand("StartWRight");
    
    /* 
    // MiddleCollectNotes, Shoots first note, Grabs left close note,
    // shoots, goes to middle, shoots far left note towards our field,
    // Intakes next middle note)
    */
    // return drivebase.getAutonomousCommand("MiddleCollectNotes");

    /*
     * MiddleCollectNotes Right, Starts at Right, grabs right close note,
     * shoots at speaker, goes to middle, intakes far right middle note,
     * 
     */

    //  return drivebase.getAutonomousCommand("MoveForwardCenter");

    return drivebase.getAutonomousCommand("Test");
  }

  public void setDriveMode() {
    // drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
}
