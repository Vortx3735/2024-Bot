// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;


import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSub extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  static CANSparkMax ArmNeo1;
  static CANSparkMax ArmNeo2;

  public ArmSub(int ID1, int ID2) {
    ArmNeo1 = new CANSparkMax(ID1, MotorType.kBrushless);
    ArmNeo2 = new CANSparkMax(ID2, MotorType.kBrushless);

    ArmNeo2.follow(ArmNeo1, true);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */

  public void move(double percentSpeed){
    ArmNeo1.set(percentSpeed);
  }

  
  public CommandBase exampleMethodCommand() {
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
