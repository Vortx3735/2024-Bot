// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;

public class ClimbSub extends SubsystemBase {

  static CANSparkMax ClimbNeo1;
  static CANSparkMax ClimbNeo2;

  private PIDController hold;

  private int setpoint;


  /** Creates a new ExampleSubsystem. */
  public ClimbSub(int ID) {
    ClimbNeo1 = new CANSparkMax(1, MotorType.kBrushless);
    ClimbNeo2 = new CANSparkMax(1, MotorType.kBrushless);

    ClimbNeo2.follow(ClimbNeo1, true);

  }

  public void move(double percentSpeed){
    ClimbNeo1.set(percentSpeed);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
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

  public void hold() {
    double pos = ClimbNeo1.getEncoder().getPosition();
    ClimbNeo1.set(hold.calculate(pos, setpoint));

    setpoint = (int)(pos);
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
