// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Intake extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
 static CANSparkMax intakeNeo1;
 private RelativeEncoder intakeEncoder;
//  private Rev2mDistanceSensor ringDetector;
 private double intakeSetPoint = 1000;
 private DigitalInput beamBreakOvershoot = new DigitalInput(1);
 private DigitalInput beamBreakNote = new DigitalInput(2);
//  private SparkLimitSwitch beamBreakOvershoot;
//  private SparkLimitSwitch beamBreakNote;
 public static Boolean hasRing = false;
 public static boolean intakingRing = false;

  public Intake(int id) {
    // ringDetector = new Rev2mDistanceSensor(Port.kOnboard);
    intakeNeo1 = new CANSparkMax(id, MotorType.kBrushless);
    intakeNeo1.setInverted(false);
    intakeNeo1.setIdleMode(IdleMode.kBrake);
    // beamBreakOvershoot = intakeNeo1.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);
    // beamBreakNote = intakeNeo1.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);


    intakeEncoder = intakeNeo1.getEncoder();

   
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

  public BooleanSupplier getOvershootBeam() {
    return () -> hasRing;
  }

  public BooleanSupplier getNoteBeam() {
    return () -> intakingRing;
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
    if(beamBreakOvershoot.get() == false){
      ringTrue();
    } else { 
      ringFalse();
    }

    if(beamBreakNote.get() == false) {
      intakingRing = true;
    } else {
      intakingRing = false;
    }

    SmartDashboard.putBoolean("intake//Beam Break Overshoot", beamBreakOvershoot.get()); //.isPressed()
    SmartDashboard.putBoolean("intake//Beam Break Overshoot has Ring", hasRing); //.isPressed()
    SmartDashboard.putBoolean("intake//Beam Break Intaking", beamBreakNote.get()); //.isPressed()

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
