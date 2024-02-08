// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  static CANSparkMax ShooterNeo1;
  static CANSparkMax ShooterNeo2;
  // SHOOTER NEO 1 = Top Roller
  // SHOOTER NEO 2 = Bottom Roller

  public Shooter(int topMotor, int bottomMotor) {
    ShooterNeo1 = new CANSparkMax(topMotor, MotorType.kBrushless);
    ShooterNeo2 = new CANSparkMax(bottomMotor, MotorType.kBrushless);


    ShooterNeo2.follow(ShooterNeo1, false);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */

  public void move(double percentSpeed){
    ShooterNeo1.set(percentSpeed);
  }

  public void shoot(double percentSpeed) {
    move(Math.abs(percentSpeed));
  }

  /**
   * why would you even want to reverse the shooter
   * @param percentSpeed
   */
  public void reverse(double percentSpeed) {
    move(Math.abs(percentSpeed));
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
