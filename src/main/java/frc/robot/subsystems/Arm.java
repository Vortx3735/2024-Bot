// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import frc.robot.Constants;




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
  public double position = 0;
  private double raw_position;
  private double offset = .3;


  public Arm(int leftMotor, int rightMotor, double motorToArmGearRatio) {
    
    //1 is left and 2 is right

    this.ArmNeo1 = new CANSparkMax(leftMotor, MotorType.kBrushless);
    this.ArmNeo2 = new CANSparkMax(rightMotor, MotorType.kBrushless);

    ArmNeo1.setInverted(false);
    ArmNeo1.setIdleMode(IdleMode.kBrake);
    ArmNeo2.setIdleMode(IdleMode.kBrake);
    ArmNeo2.follow(ArmNeo1, false);

    // armEncoder.reset();
    armEncoder.setPositionOffset(offset);

    // Arm Feed Forward
    ka = 0.0;
    kg = 0.0;
    ks = 0.0;
    kv = 0.0;

    // SmartDashboard.putNumber("arm//Arm ka (feedforward)", ka);
    // SmartDashboard.putNumber("arm//Arm kg (feedforward)", kg);
    // SmartDashboard.putNumber("arm//Arm ks (feedforward)", ks);
    // SmartDashboard.putNumber("arm//Arm kv (feedforward)", kv);

    armFF = new ArmFeedforward(ks, kg, kv, ka);

    // PID Controller

    kp = 0.01;
    ki = 0.0;
    kd = 0.0;
  
    // SmartDashboard.putNumber("arm//Arm kp (PID)", kp);
    // SmartDashboard.putNumber("arm//Arm ki (PID)", ki);
    // SmartDashboard.putNumber("arm//Arm kd (PID)", kd);

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
    if(position < .24)
    move(percentSpeed);
  }

  public void coast() {
    move(0);
  }

  public void down(double percentSpeed) {
    if(position > .025) {
      move(-percentSpeed);
    } else {
      move(0);
    }
  }


  public void moveToSetpoint(double setPointPos, double p) {
    move((setPointPos - getArmPos()) * p);
  }

  public void hold() {
    // double pos = armEncoder.getAbsolutePosition();
    setpoint = (int)position;
    ArmNeo1.set(hold.calculate(position * 2 * Math.PI, setpoint * 2 * Math.PI) + armFF.calculate(position * 2 * Math.PI, kv));
  }

  public double getArmPos() {
    // double averageNeoRotations = (ArmNeo1.getEncoder().getPosition() + ArmNeo2.getEncoder().getPosition())/2;
    // double armRotation = averageNeoRotations * motorToArmGearRatio;
    // double armAngleDegrees = armRotation * 360;
    // return armAngleDegrees;
    return position;
  }

  public BooleanSupplier getArmDown() {
    return () -> position <= Constants.ArmConstants.groundArmPos + .1 ;
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
    raw_position = armEncoder.getAbsolutePosition();
    if(raw_position < offset){
      position = 1 + raw_position - offset;
    } else {
      position = raw_position - offset;
    }
    SmartDashboard.putNumber("arm//ArmEncoder", position);
    //SmartDashboard.putNumber("arm//ArmEncoder without math", armEncoder.getAbsolutePosition());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
