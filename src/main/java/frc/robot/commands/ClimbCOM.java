// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.ClimbSub;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.DigitalInput;


/** An example command that uses an example subsystem. */
public class ClimbCOM extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ClimbSub climb;

  private DigitalInput limitSwitchTop = new DigitalInput(0);
  private DigitalInput limitSwitchBottom = new DigitalInput(1);

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ClimbCOM(ClimbSub inputClimb) {
    climb = inputClimb;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climb);
  }

  public void reverseMotor() {
    if(limitSwitchBottom.get()) {
      climb.move(RobotContainer.con1.getLeftX());
    } else {
      climb.hold();
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
