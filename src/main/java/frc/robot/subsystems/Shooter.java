// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private CANSparkMax shooterNeo1;
  private CANSparkMax shooterNeo2;
  private SparkPIDController shooterPIDController;
  private RelativeEncoder shooterEncoder;
  private  double maxRPM = 5700;
    private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

  // SHOOTER NEO 1 = Top Roller
  // SHOOTER NEO 2 = Bottom Roller`     
  public Shooter(int topMotor, int bottomMotor) {
    shooterNeo1 = new CANSparkMax(topMotor, MotorType.kBrushless);
    shooterNeo2 = new CANSparkMax(bottomMotor, MotorType.kBrushless);


    shooterNeo2.follow(shooterNeo1, false);
    shooterPIDController = shooterNeo1.getPIDController();
    shooterEncoder = shooterNeo1.getEncoder();

    // shooterPIDController.setsetPoint(0.0);
    // addChild("shooter/Shooter PID Controller", shooterPIDController); // send values to SmartDashboard
    // SmartDashboard.putNumber("Shooter P", )

  /** 
   * Example command factory method.
   *
   * @return a command
   */
  kP = 6e-5; 
  kI = 0;
  kD = 0; 
  kIz = 0; 
  kFF = 0.000015; 
  kMaxOutput = 1; 
  kMinOutput = -1;

  // set PID coefficients
  shooterPIDController.setP(kP);
  shooterPIDController.setI(kI);
  shooterPIDController.setD(kD);
  shooterPIDController.setIZone(kIz);
  shooterPIDController.setFF(kFF);
  shooterPIDController.setOutputRange(kMinOutput, kMaxOutput);

      // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("shooter/P Gain", kP);
    SmartDashboard.putNumber("shooter/I Gain", kI);
    SmartDashboard.putNumber("shooter/D Gain", kD);
    SmartDashboard.putNumber("shooter/I Zone", kIz);
    SmartDashboard.putNumber("shooter/Feed Forward", kFF);
    SmartDashboard.putNumber("shooter/Max Output", kMaxOutput);
    SmartDashboard.putNumber("shooter/Min Output", kMinOutput);
      
    SmartDashboard.putNumber("shooter/Velocity", shooterEncoder.getVelocity());
}
  public void shoot(double axis) {
    double setPoint = (axis + 1) / 2*maxRPM;
    if (setPoint>maxRPM) {
      setPoint=maxRPM;
    }
    shooterPIDController.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
    System.out.println("Shooting" + setPoint + "Encoder Velocity" + shooterEncoder.getVelocity());
  }


  public void coast() {
    shooterNeo1.set(0);
  }

  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
