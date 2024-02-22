// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;


public class AdvantageScope extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
    // advantage scope stuff
  public StructPublisher<Pose2d> publisher;
  public StructArrayPublisher<Pose2d> arrayPublisher;

  public Pose2d poseA;

  public final Field2d m_field = new Field2d();

     
  public AdvantageScope() {
    // advantagescope stuff
    poseA = RobotContainer.drivebase.getSwervePose();

    // WPILib
    publisher = NetworkTableInstance.getDefault()
        .getStructTopic("MyPose", Pose2d.struct).publish();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // advantage scope stuff
    publisher.set(m_field.getRobotPose());
    SmartDashboard.putData("Field", m_field);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
