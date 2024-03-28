// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private CANSparkMax shooterNeo1;
  private CANSparkMax shooterNeo2;
  private SparkPIDController m_pidController;
  private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  // SHOOTER NEO 1 = Top Roller
  // SHOOTER NEO 2 = Bottom Roller`     
  public Shooter(int topMotor, int bottomMotor) {
    shooterNeo1 = new CANSparkMax(topMotor, MotorType.kBrushless);
    shooterNeo2 = new CANSparkMax(bottomMotor, MotorType.kBrushless);


    // shooterNeo2.follow(shooterNeo1, false);
    shooterNeo1.setIdleMode(IdleMode.kBrake);
    shooterNeo2.setIdleMode(IdleMode.kBrake);
    shooterNeo1.getEncoder();
    m_pidController = shooterNeo1.getPIDController();


    // PID coefficients
    kP = 0.0004;
    kI = 0;
    kD = 0;
    kIz = 0;
    kFF = 0.00017; // .000015
    kMaxOutput = 1;
    kMinOutput = -1;

    // set PID coefficients
    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);

      
    // SmartDashboard.putNumber("shooter/Velocity", shooterEncoder.getVelocity());
  }


  public void setBrakeMode(IdleMode mode) {
    shooterNeo1.setIdleMode(mode);
    shooterNeo2.setIdleMode(mode);
  }

  public void move(double speed) {
    shooterNeo1.set(speed);
    shooterNeo2.set(-speed);
  }

  

  public void setShooterRPM(double input) {
    if (input < ShooterConstants.maxRPM) {
        m_pidController.setReference(-input, ControlType.kVelocity);
    } else {
        System.out.println("Shooter Motor Warning - Cannot Set Motor RPM Over Limit Of "
                + ShooterConstants.maxRPM);
    }
  }
  
  public void coast() {
    shooterNeo1.set(0);
    shooterNeo2.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //SmartDashboard.putNumber("shooter// Shooter Bottom RPM", shooterNeo2.getEncoder().getVelocity());
    //SmartDashboard.putNumber("shooter// Shooter Top RPM", shooterNeo1.getEncoder().getVelocity());
    //SmartDashboard.putNumber("shooter//Shooter Voltage", shooterNeo1.getBusVoltage());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
