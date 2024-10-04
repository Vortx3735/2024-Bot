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
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Climb;
import frc.robot.util.VorTXControllerXbox;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;

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
  
  public static LED led = new LED(0, 18);

  public static Climb climb = new Climb(10, 11);
  public static ClimbCom climbcom = new ClimbCom(climb);
  

  public static final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve"));


  public static AdvantageScope advantagescope = new AdvantageScope();
      
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

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
        intakecom.intakeNoteCom()
    );

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

    con2.lt.whileTrue(
      new RunCommand(
        () -> shooter.move(.80), 
        shooter)
    );

    con2.rt.whileTrue(
      new RunCommand(
        () -> shooter.move(.80), 
        shooter).alongWith(
          new RunCommand(
            () -> intake.move(.80), 
            intake)
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

    // yes ayman i know that its not technically part of the arm but idc shut up ur a nerd
    // + ur driving sucks and u lose us state
    /*
     * 
     * 
     * 
     * 
     * 
     * 
     * 
     * 
     * 
     */
    // con1.view.onTrue(
    //   new InstantCommand(
    //     () -> arm.switchCam(),
    //     arm
    //   )
    // );

    // con2.aButton.whileTrue(
    //   autoaim
    // );


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

    // RunCommand moveArmToShot = new RunCommand(
    //   () -> arm.moveToSetpoint(Constants.ArmConstants.armTravelPos, 4), 
    //   arm
    //   );

    // con2.aButton.whileTrue(
    //   moveArmToShot
    // );

    con1.lb.toggleOnTrue(
      new InstantCommand(
        () -> drivebase.togglePrecisionMode(),
        drivebase)
      );

    con1.rb.toggleOnTrue(
      new InstantCommand(
        () -> drivebase.toggleTrackSpeakerMode(),
      drivebase)
    );

    // con2.aButton.whileTrue(
    //   RobotContainer.climbcom.getMoveCommand(-.25)
    // );

    con2.povUp.whileTrue(
      new RunCommand(
        () -> climb.moveRight(-.25), 
        climb)
    );

    con2.povDown.whileTrue(
      new RunCommand(
        () -> climb.moveRight(.25), 
        climb)
    );

    // con1.povUp.whileTrue(
    //   new RunCommand(
    //     () -> climb.moveLeft(.25), 
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
   * Use this to pass the  fnomous command to the main {@link Robot} class.
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

    // return drivebase.getAutonomousCommand("RightMiddleCollectNotes");

     return drivebase.getAutonomousCommand("FourNoteCenter");

    // return drivebase.getAutonomousCommand("Static Shot");

    
  }

  public void setDriveMode() {
    // drivebase.setDefaultCommand();
  }

  

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
}
