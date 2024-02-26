// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.nio.ReadOnlyBufferException;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;
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
  }

  public Command shooterIdle() {
    if(RobotContainer.intake.getRing() == true){
      return new RunCommand(
        () -> RobotContainer.shooter.move(.5), 
        RobotContainer.shooter);
    } else {
      return new RunCommand(
        () -> RobotContainer.shooter.move(0),
        RobotContainer.shooter);
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
      () -> RobotContainer.arm.down(.5), 
      RobotContainer.arm).withTimeout(.2).andThen(
        shootFromSub()
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