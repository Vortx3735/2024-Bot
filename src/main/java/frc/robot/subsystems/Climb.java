// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private CANSparkMax climbNeo1;
  private CANSparkMax climbNeo2;
  private RelativeEncoder climbEncoder;

  // SHOOTER NEO 1 = Top Roller
  // SHOOTER NEO 2 = Bottom Roller`     
  public Climb(int leftMotor, int rightMotor) {
    climbNeo1 = new CANSparkMax(leftMotor, MotorType.kBrushless);
    climbNeo2 = new CANSparkMax(rightMotor, MotorType.kBrushless);

    climbNeo1.setIdleMode(IdleMode.kCoast);    
    climbNeo2.setIdleMode(IdleMode.kCoast);

    climbNeo2.follow(climbNeo1);

    climbEncoder = climbNeo1.getEncoder();
  }

  public void move(double speed) {
    climbNeo1.set(speed);
  }


  public void coast() {
    climbNeo1.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("climb/Climb RPM", climbEncoder.getVelocity()); // Climb Velocity
    SmartDashboard.putNumber("climb/Climb Voltage", climbNeo1.getBusVoltage()); // Climb Voltage
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
