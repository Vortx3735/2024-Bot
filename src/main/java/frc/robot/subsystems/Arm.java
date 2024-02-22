// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.simulation.EncoderSim;




public class Arm extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  CANSparkMax ArmNeo1;
  CANSparkMax ArmNeo2;

  private PIDController hold;
  private double ki, kp, kd;

  private ArmFeedforward armFF;
  private double ka, kg, ks, kv;

  private int setpoint;
  // private final double motorToArmGearRatio;

  private final DutyCycleEncoder armEncoder = new DutyCycleEncoder(0);


  public Arm(int leftMotor, int rightMotor, double motorToArmGearRatio) {
    this.ArmNeo1 = new CANSparkMax(leftMotor, MotorType.kBrushless);
    this.ArmNeo2 = new CANSparkMax(rightMotor, MotorType.kBrushless);

    ArmNeo1.setInverted(true);
    ArmNeo1.setIdleMode(IdleMode.kBrake);
    ArmNeo2.setIdleMode(IdleMode.kBrake);
    this.ArmNeo2.follow(ArmNeo1, true);

    // Arm Feed Forward
    ka = 0.0;
    kg = 0.0;
    ks = 0.0;
    kv = 0.0;

    SmartDashboard.putNumber("arm//Arm ka (feedforward)", ka);
    SmartDashboard.putNumber("arm//Arm kg (feedforward)", kg);
    SmartDashboard.putNumber("arm//Arm ks (feedforward)", ks);
    SmartDashboard.putNumber("arm//Arm kv (feedforward)", kv);

    armFF = new ArmFeedforward(ks, kg, kv, ka);

    // PID Controller

    kp = 0.01;
    ki = 0.0;
    kd = 0.0;
  
    SmartDashboard.putNumber("arm//Arm kp (PID)", kp);
    SmartDashboard.putNumber("arm//Arm ki (PID)", ki);
    SmartDashboard.putNumber("arm//Arm kd (PID)", kd);

    hold = new PIDController(kp, ki, kd);



    this.setpoint = 0;
    // this.motorToArmGearRatio = motorToArmGearRatio;
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
    double pos = armEncoder.getAbsolutePosition();
    ArmNeo1.set(hold.calculate(pos, setpoint) + armFF.calculate(pos, kv));

    setpoint = (int)(pos);

    

  }

  public double getArmAngle() {
    // double averageNeoRotations = (ArmNeo1.getEncoder().getPosition() + ArmNeo2.getEncoder().getPosition())/2;
    // double armRotation = averageNeoRotations * motorToArmGearRatio;
    // double armAngleDegrees = armRotation * 360;
    // return armAngleDegrees;
    return armEncoder.getAbsolutePosition();
  }

  public void setArmBrake(IdleMode mode) {
    ArmNeo1.setIdleMode(mode);
    ArmNeo2.setIdleMode(mode);
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
