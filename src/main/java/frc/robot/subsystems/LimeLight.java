// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//import java.lang.runtime.SwitchBootstraps;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants.VisionConstants;

public class LimeLight extends SubsystemBase {
  /** Creates a new ShooterVision. */
  private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  private NetworkTableEntry tx = table.getEntry("tx");
  private NetworkTableEntry ty = table.getEntry("ty");
  private NetworkTableEntry ta = table.getEntry("ta");
  private NetworkTableEntry tid = table.getEntry("tid");
  private boolean m_targeting = false;

  public LimeLight() {
    /**
     * tx - Horizontal Offset
     * ty - Vertical Offset 
     * ta - Area of target 
     * tv - Target Visible
     */

    
    this.tx = table.getEntry("tx");
    this.ty = table.getEntry("ty");
    this.ta = table.getEntry("ta");
    this.tid = table.getEntry("tid");
  }

  public int getID() {
    return (int)tid.getDouble(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //read values periodically
    double x = this.tx.getDouble(0.0);
    double y = this.ty.getDouble(0.0);
    double area = this.ta.getDouble(0.0);
    int id = getID();
  


    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);

  }

    /**Returns distance to target in inches */
  public double distanceToTargetinMeter(){
    double cameraAngle = VisionConstants.limelightDegrees; 
    double angleToTarget = this.tx.getDouble(0.0);
    double camHeight = VisionConstants.heightFromGround;
    double targetHeight = switch (getID()) {
      case 1, 2, 5, 6, 9, 10 -> 0.122 + 0.0135;
      case 3, 4, 7, 8 -> 0.132 + 0.0135;
      case 11, 12, 13, 14, 15, 16 -> 0.121 + 0.0115;
      default -> 0;
    };
    double distance =  ((targetHeight-camHeight) / Math.tan(cameraAngle+angleToTarget));
    return distance;
    
  }


  public NetworkTableEntry getTx() {
    return tx;
  }

  public NetworkTableEntry getTy() {
    return ty;
  }

  public NetworkTableEntry getTa() {
    return ta;
  }

  public void setPipeline(int pipeline){
    this.table.getEntry("pipeline").setNumber(pipeline);
  }
  
  public boolean isTargeting(){
    return m_targeting;
  }

  public void setTargeting(boolean targeting){
	  //setLedOn(targeting);
	  m_targeting = targeting;

  }
}