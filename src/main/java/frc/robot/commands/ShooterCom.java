// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

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

  public void shoot() {
      if(Intake.hasRing) {
        
        new ParallelCommandGroup(
          new RunCommand(
            () -> shooter.move(.75), 
            shooter
            ),
          new SequentialCommandGroup(
            new WaitCommand(2),
            new RunCommand(
              () -> RobotContainer.intake.move(.25), 
              RobotContainer.intake
              )
          )
        );
      }
    }

    public Command shootFromSub() {
        return  new SequentialCommandGroup(
            new RunCommand(
              () -> shooter.move(1), // rev up shooter
              shooter).withTimeout(2),
            
            new RunCommand(
              () -> RobotContainer.intake.move(1), 
              RobotContainer.intake).alongWith(
                new RunCommand(
                  () -> shooter.move(1), // shooter
                  shooter)
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
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}