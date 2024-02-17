// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Intake;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.*;


/** An example command that uses an example subsystem. */
public class IntakeCom extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Intake intake;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */

   
  public IntakeCom(Intake i) {
    intake = i;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  public void intakeNote() {
    intake.move(.4);
  }

  public void fixOvershoot() {
    intake.move(-.2);
  }

  public Command intakeNoteCom() {
    return new SequentialCommandGroup(
      new RunCommand(
        () -> RobotContainer.intake.move(.4), 
        RobotContainer.intake
        ).until(RobotContainer.intake.getBeam()),
        
      new RunCommand(
          () -> RobotContainer.intake.move(-.2), 
          RobotContainer.intake
        ).until(RobotContainer.intake.getDeadBeam()),

      new RunCommand(
        () -> RobotContainer.intake.move(.1), 
        RobotContainer.intake).withTimeout(.3)
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