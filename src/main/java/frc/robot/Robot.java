// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.io.IOException;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import swervelib.parser.SwerveParser;



/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static Robot instance;
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
     


  // private DigitalInput beamBreak = new DigitalInput(IntakeConstants.BEAM_BREAK_PORT);


  public Robot() {
    instance = this;
  }

  public static Robot getInstance() {
    return instance;
  }
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  double value = 19;

  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    RobotContainer.drivebase.swerveDrive.setMotorIdleMode(true);

    // CameraServer.startAutomaticCapture();

    enableLiveWindowInTest(true); //enabling test mode
    SmartDashboard.putData(CommandScheduler.getInstance());

    // Create a timer to disable motor brake a few seconds after disable.  This will let the robot stop
    // immediately when disabled, but then also let it be pushed more 
    // SmartDashboard.putNumber("shooter/Shooter Setpoint", value);

    // update swerve pose in advantagescope field
    RobotContainer.advantagescope.m_field.setRobotPose(RobotContainer.drivebase.getSwervePose());

    for (int port = 5800; port <= 5807; port++) {
      PortForwarder.add(port, "limelight.local", port);
    }

  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    //value = SmartDashboard.getNumber("shooter/Shooter Setpoint", 1.0);
    //SmartDashboard.putNumber("shooter/Shooter Setpoint 2", value);
    // SmartDashboard.putBoolean("intake/Beam Break",beamBreak.get());
    
    SmartDashboard.putNumber("LeftX", RobotContainer.con1.getLeftX());
    SmartDashboard.putNumber("LeftY", RobotContainer.con1.getLeftY());
    SmartDashboard.putNumber("RightX", RobotContainer.con1.getRightX());
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    RobotContainer.arm.setArmBrake(IdleMode.kCoast);

    // m_robotContainer.setMotorBrake(true);
    // disabledTimer.reset();
    // disabledTimer.start();
  }

  @Override
  public void disabledPeriodic() {
    RobotContainer.drivebase.setPresisionModeFalse();
    // if (disabledTimer.hasElapsed(Constants.Drivebase.WHEEL_LOCK_TIME))
    // {
    //   // m_robotContainer.setMotorBrake(false);
    //   // disabledTimer.stop();
    // }
    RobotContainer.led.vorTXStreak();
    // RobotContainer.led.noteCheck();
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {

    CommandScheduler.getInstance().cancelAll();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    RobotContainer.led.rainbow();
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    // if (m_autonomousCommand != null)
    // {
    //   m_autonomousCommand.cancel();
    // }
    // m_robotContainer.setDriveMode();
    // m_robotContainer.setMotorBrake(true);
    RobotContainer.arm.setArmBrake(IdleMode.kBrake);
    RobotContainer.drivebase.setMotorBrake(true);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    if(RobotContainer.con2.rt.getAsBoolean() || RobotContainer.con2.rb.getAsBoolean()) {
      RobotContainer.led.blinkColor(Color.kRed);
    } else if (RobotContainer.con1.rt.getAsBoolean() || RobotContainer.con1.lt.getAsBoolean()) {
      RobotContainer.led.blinkColor(Color.kBlue);
    } else if (RobotContainer.con2.lt.getAsBoolean()) {
      RobotContainer.led.blinkColor(Color.kGreen);
    } else {
      RobotContainer.led.noteCheck();
    }
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    try
    {
      new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve"));
    } catch (IOException e)
    {
      throw new RuntimeException(e);
    }
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {

  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {

  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    
  }
}
