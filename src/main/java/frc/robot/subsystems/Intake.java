// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;


public class Intake extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
 static CANSparkMax intakeNeo1;
 private RelativeEncoder intakeEncoder;
//  private Rev2mDistanceSensor ringDetector;
 private double intakeSetPoint = 1000;
 private DigitalInput beamBreak = new DigitalInput(IntakeConstants.BEAM_BREAK_PORT);
 public static boolean hasRing = false;

  public Intake(int id) {
    // ringDetector = new Rev2mDistanceSensor(Port.kOnboard);
    intakeNeo1 = new CANSparkMax(id, MotorType.kBrushless);
    intakeNeo1.setInverted(false);
    intakeNeo1.setIdleMode(IdleMode.kBrake);

    intakeEncoder = intakeNeo1.getEncoder();

    SmartDashboard.putBoolean("intake//Beam Break", beamBreak.get());


  
      
    SmartDashboard.putNumber("ProcessVariable", intakeEncoder.getVelocity());

    SmartDashboard.getNumber("intake/Intake Setpoint", intakeSetPoint);
    // SmartDashboard.putBoolean("intake/Beam Break",getBeam());

  }
  /**
   * Example command factory method.
   *
   * @return a command
   */

  
  public void move(double speed) {
    intakeNeo1.set(speed);
  }

  public void stopIntake() {
    intakeNeo1.set(0);
  }

  public BooleanSupplier getDeadBeam() {
    return () -> beamBreak.get();
  }

  public BooleanSupplier getBeam() {
    return () -> !beamBreak.get();
  }

  public boolean getRing() {
    return hasRing;
  }

  public void ringTrue() {
    hasRing = true;
  }

  public void ringFalse() {
    hasRing = false;
  }

  // private double getDistance(){
  //   return ringDetector.GetRange();
  // }

  // public boolean checkRing(double dist, double error) {
  //   return getDistance() < dist - error;
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
