package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;

public class FaceAprilTagCommand extends Command {
  private final SwerveSubsystem swerveSubsystem;
  private final LimeLight visionSubsystem; // Assuming you have a subsystem for vision processing
  private final Optional<Alliance> alliance;
  private PIDController rotationController;
  private double kp, ki, kd;
  private int setpoint;
  


  public FaceAprilTagCommand(SwerveSubsystem swerveSubsystem, LimeLight visionSubsystem) {
    this.swerveSubsystem = swerveSubsystem;
    this.visionSubsystem = visionSubsystem;
    addRequirements(swerveSubsystem);

    alliance = DriverStation.getAlliance();

    kp = 0.0;
    ki = 0.0;
    kd = 0.0; 
    rotationController = new PIDController(kp, ki, kd);
  }

  public void alignRobot() { 
    double tx = visionSubsystem.getTx().getDouble(0); // Fetch the horizontal offset
    double rotationSpeed = rotationController.calculate(tx, 0); // Calculate the rotation speed needed to align
    
    swerveSubsystem.drive(new Translation2d(0, 0), rotationSpeed, false); // Rotate the robot
  }


  @Override
  public void initialize() {
    // Initialization logic here
  }

  @Override
  public void execute() {
    // Here, you would use visionSubsystem to detect the AprilTag and calculate the rotation needed
    // Then, command the swerveSubsystem to rotate accordingly

  }

  @Override
  public boolean isFinished() {
    // Determine condition for command completion, possibly checking if the robot is facing the AprilTag within a margin of error
    return false;
  }
}
