// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;

/** An example command that uses an example subsystem. */
public class ShooterCom extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Shooter shooter;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */

   
  public ShooterCom(Shooter s) {
    shooter = s;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
    addRequirements(RobotContainer.intake);
  }

  public Command shooterIdle() {
      return new RunCommand(
        () -> idleShot(), 
        RobotContainer.shooter);
  }

  public void idleShot() {
    if(RobotContainer.intake.getRing()) {
      RobotContainer.shooter.move(.5);
    } else {
      RobotContainer.shooter.move(0);
    }
  }

  // public void shoot() {
  //     if(Intake.hasRing) {
        
  //       new ParallelCommandGroup(
  //         new RunCommand(
  //           () -> shooter.move(.75), 
  //           shooter
  //           ),
  //         new SequentialCommandGroup(
  //           new WaitCommand(2),
  //           new RunCommand(
  //             () -> RobotContainer.intake.move(.25), 
  //             RobotContainer.intake
  //             )
  //         )
  //       );
  //     }
  //   }

  public Command firstShotFromSub() {
    return new RunCommand(
      () ->
        RobotContainer.arm.moveToSetpoint(Constants.ArmConstants.groundArmPos, 4), 
        RobotContainer.arm
      ).until(RobotContainer.arm.getArmDown()).andThen(
        shootFromSub()
    );
  }

  public Command firstShotFromSubasdf() {
    return  new RunCommand(
        () ->
          RobotContainer.arm.moveToSetpoint(Constants.ArmConstants.groundArmPos, 4), 
          RobotContainer.arm
      ).until(RobotContainer.arm.getArmDown()).alongWith(
        new RunCommand(() -> RobotContainer.shooter.move(1), 
      RobotContainer.shooter).withTimeout(2)).andThen(
        shootFromSubasdf()
    );
  }

  public Command shootFromSubasdf() {
        return  new SequentialCommandGroup(
            // new RunCommand(
            //   () -> RobotContainer.shooter.move(1), // rev up shooter
            //   RobotContainer.shooter).withTimeout(2),
            
            new RunCommand(
              () -> RobotContainer.intake.move(1), 
              RobotContainer.intake).alongWith(
                new RunCommand(
                  () -> RobotContainer.shooter.move(1), // shooter
                  RobotContainer.shooter)
              )
          );
    }

    public Command shootFromSub() {
        return  new SequentialCommandGroup(
            new RunCommand(
              () -> RobotContainer.shooter.move(1), // rev up shooter
              RobotContainer.shooter).withTimeout(2),
            
            new RunCommand(
              () -> RobotContainer.intake.move(1), 
              RobotContainer.intake).alongWith(
                new RunCommand(
                  () -> RobotContainer.shooter.move(1), // shooter
                  RobotContainer.shooter)
              )
          );
    }

    public Command shootFromSubOther() {
        return  new SequentialCommandGroup(
            new RunCommand(
              () -> RobotContainer.shooter.move(0.8), // rev up shooter
              RobotContainer.shooter).withTimeout(.5),
            
            new RunCommand(
              () -> RobotContainer.intake.move(1), 
              RobotContainer.intake).alongWith(
                new RunCommand(
                  () -> RobotContainer.shooter.move(0.8), // shooter
                  RobotContainer.shooter)
              )
          );
    }

    public Command autonShooterIdle() {
      return new RunCommand(
          () -> RobotContainer.shooter.move(1),
          RobotContainer.shooter
      );
    }

    // public Command moveArmUp(){
    //   return new RunCommand(
    //     () -> RobotContainer.arm.up(.5), 
    //     RobotContainer.arm).withTimeout(1).andThen(
    //       shootFromSub()
    //     );
    // }

    // public Command moveArmDown(){
    //   return new RunCommand(
    //     () -> RobotContainer.arm.down(.5), 
    //     RobotContainer.arm).withTimeout(1);
    // }


    /**
   * 
   *     /_/_/_/_/    /_/_/_/_/_/    /_/     /_/        /_/          /_/_/     /_/         
   *     /_/              /_/        /_/     /_/       /_//_/        /_/ /_/   /_/
   *     /_/_/_/_/        /_/        /_/_/_/_/_/      /_/  /_/       /_/   /_/ /_/
   *     /_/              /_/        /_/     /_/     /_/_/_/_/_/     /_/    /_//_/
   *     /_/_/_/_/        /_/        /_/     /_/    /_/      /_/     /_/       /_/
   * 
   * 
   *     /_/          /_/_/_/_/    /_/_/_/_/
   *     /_/          /_/          /_/
   *     /_/          /_/_/_/_/    /_/_/_/_/
   *     /_/          /_           /_/
   *     /_/_/_/_/    /_/_/_/_/    /_/_/_/_/
   * 
   * 
   *     /_/       /_/     /_/      /_/
   *      /_/     /_/_/   /_/      /_/
   *       /_/  /_/  /_/ /_/      /_/
   *        /_/_/     /_/_/      /_/
   *         /_/       /_/      /_/
   * 
   * 
   */

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

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