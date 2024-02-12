// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.controller.PIDController;

public class Arm extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  CANSparkMax ArmNeo1;
  CANSparkMax ArmNeo2;

  private PIDController hold;

  private int setpoint;
  private final double motorToArmGearRatio;


  public Arm(int leftMotor, int rightMotor, double motorToArmGearRatio) {
    this.ArmNeo1 = new CANSparkMax(leftMotor, MotorType.kBrushless);
    this.ArmNeo2 = new CANSparkMax(rightMotor, MotorType.kBrushless);

    ArmNeo1.setInverted(true);
    this.ArmNeo2.follow(ArmNeo1, true);

    this.hold = new PIDController(0.01, 0, 0);

    this.setpoint = 0;
    this.motorToArmGearRatio = motorToArmGearRatio;
  }
  
  /**
   * Example command factory method.
   *
   * @return a command
   */



  private void move(double percentSpeed){
    ArmNeo1.set(percentSpeed);
    System.out.println("Setting Speed" + percentSpeed);
  }


  //add soft limits based on encoder position
  public void up(double percentSpeed) {
    move(percentSpeed);
  }

  public void coast() {
    move(0);
  }

  public void down(double percentSpeed) {
    move(-percentSpeed);
  }


  public void moveToSetpoint(double setPointDegrees, double p) {
    move((setPointDegrees - getArmAngle()) * p);
  }

  public void hold() {
    double pos = ArmNeo1.getEncoder().getPosition();
    ArmNeo1.set(hold.calculate(pos, setpoint));

    setpoint = (int)(pos);
  }

  public double getArmAngle() {
    double averageNeoRotations = (ArmNeo1.getEncoder().getPosition() + ArmNeo2.getEncoder().getPosition())/2;
    double armRotation = averageNeoRotations * motorToArmGearRatio;
    double armAngleDegrees = armRotation * 360;
    return armAngleDegrees;
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
