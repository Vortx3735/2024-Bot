// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.CANSparkLowLevel.MotorType;

import com.revrobotics.Rev2mDistanceSensor.Port;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;






public class Intake extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  static CANSparkMax intakeNeo1;

  private Rev2mDistanceSensor ringDetector;

  public Intake(int id) {
    ringDetector = new Rev2mDistanceSensor(Port.kOnboard);
    intakeNeo1 = new CANSparkMax(id, MotorType.kBrushless);
    intakeNeo1.setInverted(true);
  }
  /**
   * Example command factory method.
   *
   * @return a command
   */


  private void move(double percentSpeed){
    intakeNeo1.set(percentSpeed);
  }

  public void intake(double percentSpeed) {
    move(Math.abs(percentSpeed));
  }

  public void outtake(double percentSpeed) {
    move(Math.abs(percentSpeed));
  }

  private double getDistance(){
    return ringDetector.GetRange();
  }

  public boolean hasRing(double dist, double error) {
    return getDistance() < dist - error;
  }

  public void coast() {
    move(0);
  }

   public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
