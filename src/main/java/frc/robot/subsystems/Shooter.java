// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private CANSparkMax shooterNeo1;
  private CANSparkMax shooterNeo2;
  private RelativeEncoder shooterEncoder;

  // SHOOTER NEO 1 = Top Roller
  // SHOOTER NEO 2 = Bottom Roller`     
  public Shooter(int topMotor, int bottomMotor) {
    shooterNeo1 = new CANSparkMax(topMotor, MotorType.kBrushless);
    shooterNeo2 = new CANSparkMax(bottomMotor, MotorType.kBrushless);


    shooterNeo2.follow(shooterNeo1, false);
    shooterNeo1.setIdleMode(IdleMode.kBrake);
    shooterNeo2.setIdleMode(IdleMode.kBrake);
    shooterEncoder = shooterNeo1.getEncoder();

      
    SmartDashboard.putNumber("shooter/Velocity", shooterEncoder.getVelocity());
}

  public void move(double speed) {
    shooterNeo1.set(speed);
  }
  
  public void coast() {
    shooterNeo1.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("shooter// Shooter Bottom RPM", shooterNeo2.getEncoder().getVelocity());
    SmartDashboard.putNumber("shooter// Shooter Top RPM", shooterNeo1.getEncoder().getVelocity());
    SmartDashboard.putNumber("shooter//Shooter Voltage", shooterNeo1.getBusVoltage());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
