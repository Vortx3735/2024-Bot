// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climb;

/** An example command that uses an example subsystem. */
public class ClimbCom extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Climb climb;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */

   
  public ClimbCom(Climb c) {
    climb = c;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climb);
  }

  // returns default command: coast
  public Command getDefaultCommand(){
    return new RunCommand(
        () -> climb.coast(),
        climb);
  }

  // returns move command, takes in speed parameter
  public Command getMoveCommand(double speed){
    return new RunCommand(
        () -> climb.move(speed),
        climb);
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