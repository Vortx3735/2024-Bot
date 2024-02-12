// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.CANSparkLowLevel.MotorType;

import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;


public class Intake extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
 static CANSparkMax intakeNeo1;
 private SparkPIDController intakePIDController;
 private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;
 private RelativeEncoder intakeEncoder;
 private Rev2mDistanceSensor ringDetector;
 private double intakeSetPoint = 1000;
 private DigitalInput beamBreak = new DigitalInput(IntakeConstants.BEAM_BREAK_PORT);

  public Intake(int id) {
    ringDetector = new Rev2mDistanceSensor(Port.kOnboard);
    intakeNeo1 = new CANSparkMax(id, MotorType.kBrushless);
    intakeNeo1.setInverted(true);

    intakePIDController = intakeNeo1.getPIDController();
    intakeEncoder = intakeNeo1.getEncoder();


    kP = 6e-5; 
    kI = 0;
    kD = 0; 
    kIz = 0; 
    kFF = 0.000015; 
    kMaxOutput = 1; 
    kMinOutput = -1;
    maxRPM = 5700;

    // set PID coefficients
    intakePIDController.setP(kP);
    intakePIDController.setI(kI);
    intakePIDController.setD(kD);
    intakePIDController.setIZone(kIz);
    intakePIDController.setFF(kFF);
    intakePIDController.setOutputRange(kMinOutput, kMaxOutput);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
      
    SmartDashboard.putNumber("ProcessVariable", intakeEncoder.getVelocity());

    SmartDashboard.getNumber("intake/Intake Setpoint", intakeSetPoint);
    // SmartDashboard.putBoolean("intake/Beam Break",getBeam());

  }
  /**
   * Example command factory method.
   *
   * @return a command
   */


  public void move(boolean isForward){
    System.out.println("moving intake" + intakeSetPoint);
    if (intakeSetPoint>maxRPM) {
      intakeSetPoint=maxRPM;
    }
    intakeSetPoint = Math.abs(intakeSetPoint);
    if(isForward){
      intakeSetPoint = -intakeSetPoint;
    }
    else{

    }
    intakePIDController.setReference(intakeSetPoint, CANSparkMax.ControlType.kVelocity);
  }

  public void stopIntake(){
    intakeNeo1.set(0);
  }

  private boolean getBeam(){
    return beamBreak.get();
  }

  private double getDistance(){
    return ringDetector.GetRange();
  }

  public boolean hasRing(double dist, double error) {
    return getDistance() < dist - error;
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
