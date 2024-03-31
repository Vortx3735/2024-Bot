// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shooter;
import frc.robot.util.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.util.RootFinder;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;


/** An example command that uses an example subsystem. */
public class AutoAim extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Arm arm;
  private final Shooter shooter;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */

   
  public AutoAim(Arm a, Shooter s) {
    arm = a;
    shooter = s;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm, shooter);
  }

  public Command aimSpeaker() {
    if(LimelightHelpers.getFiducialID("limelight") == VisionConstants.speakerMidTag) {
      double kP = 0.1;
      double xDistLLToShooter = ArmConstants.armLength*Math.cos(RobotContainer.arm.getRadiansTravelled()) 
                                - ShooterConstants.shooterLength*Math.cos(RobotContainer.arm.getRadiansTravelled() 
                                                                            + ShooterConstants.differenceFromArm)
                                + VisionConstants.limelightXDistToArmPivot;
      double yDistLLToShooter = ArmConstants.armLength*Math.sin(RobotContainer.arm.getRadiansTravelled()) 
                                - ShooterConstants.shooterLength*Math.sin(RobotContainer.arm.getRadiansTravelled()
                                                                            + ShooterConstants.differenceFromArm)
                                - VisionConstants.limelightYDistToArmPivot;
      double targetOffsetAngle_Vertical = LimelightHelpers.getTY("limelight");  
  
      double angleToGoalDegrees = VisionConstants.limelightDegrees + targetOffsetAngle_Vertical;
      double angleToGoalRadians = Units.degreesToRadians(angleToGoalDegrees);
  
      //calculate distance
      double distanceFromLimelightToGoalMeters = (FieldConstants.speakerHeight - VisionConstants.limelightHeight) / Math.tan(angleToGoalRadians);
      double xShooterToGoal = distanceFromLimelightToGoalMeters + xDistLLToShooter;
  
      double yShooterToGoal = FieldConstants.speakerHeight - ArmConstants.pivotHeight - yDistLLToShooter;
      double error = stupid(xShooterToGoal, yShooterToGoal)[0] - RobotContainer.arm.getRadiansTravelled();
      double rpmNeeded = stupid(xShooterToGoal, yShooterToGoal)[1];
  
      return new RunCommand(
        () -> RobotContainer.arm.move(error*kP),
        RobotContainer.arm
      ).alongWith(
        new RunCommand(
          () -> RobotContainer.shooter.setShooterRPM(rpmNeeded),
          RobotContainer.shooter)
      );
    } else {
      System.out.println(((DriverStation.getAlliance().get() == DriverStation.Alliance.Red) ? "Red" : "Blue") + " Alliance Mid Speaker AprilTag Not Found!");
      return new RunCommand(
        () -> RobotContainer.arm.getDefaultCommand(),
        RobotContainer.arm
      ).alongWith(
        new RunCommand(
          () -> RobotContainer.shooter.getDefaultCommand(),
          RobotContainer.shooter)
      );
    }
  }

  public Command aimSpeakerAuto() {
    if(LimelightHelpers.getFiducialID("limelight") == VisionConstants.speakerMidTag) {
      double kP = 0.1;
      double xDistLLToShooter = ArmConstants.armLength*Math.cos(RobotContainer.arm.getRadiansTravelled()) 
                                - ShooterConstants.shooterLength*Math.cos(RobotContainer.arm.getRadiansTravelled() 
                                                                            + ShooterConstants.differenceFromArm)
                                + VisionConstants.limelightXDistToArmPivot;
      double yDistLLToShooter = ArmConstants.armLength*Math.sin(RobotContainer.arm.getRadiansTravelled()) 
                                - ShooterConstants.shooterLength*Math.sin(RobotContainer.arm.getRadiansTravelled()
                                                                            + ShooterConstants.differenceFromArm)
                                - VisionConstants.limelightYDistToArmPivot;
      double targetOffsetAngle_Vertical = LimelightHelpers.getTY("limelight");  
  
      double angleToGoalDegrees = VisionConstants.limelightDegrees + targetOffsetAngle_Vertical;
      double angleToGoalRadians = Units.degreesToRadians(angleToGoalDegrees);
  
      //calculate distance
      double distanceFromLimelightToGoalMeters = (FieldConstants.speakerHeight - VisionConstants.limelightHeight) / Math.tan(angleToGoalRadians);
      double xShooterToGoal = distanceFromLimelightToGoalMeters + xDistLLToShooter;
  
      double yShooterToGoal = FieldConstants.speakerHeight - ArmConstants.pivotHeight - yDistLLToShooter;
      double error = stupid(xShooterToGoal, yShooterToGoal)[0] - RobotContainer.arm.getRadiansTravelled();
      double rpmNeeded = stupid(xShooterToGoal, yShooterToGoal)[1];
  
      return new RunCommand(
        () -> RobotContainer.arm.move(error*kP),
        RobotContainer.arm
      ).alongWith(
        new RunCommand(
          () -> RobotContainer.shooter.setShooterRPM(rpmNeeded),
          RobotContainer.shooter)
      );
    } else {
      System.out.println(((DriverStation.getAlliance().get() == DriverStation.Alliance.Red) ? "Red" : "Blue") + " Alliance Mid Speaker AprilTag Not Found!");
      return new RunCommand(
        () -> RobotContainer.arm.getDefaultCommand(),
        RobotContainer.arm
      ).alongWith(
        new RunCommand(
          () -> RobotContainer.shooter.getDefaultCommand(),
          RobotContainer.shooter)
      );
    }
  }

  public static double[] stupid(double xDist, double yDist) {
        // this code is very effective until around 30 feet which is further than podium so it doesnt matter lol
        //      the reason its uneffective at until 30 is bc the note needs to follow the trajectory and enter the speaker
        //      before the trajectory reaches its apex and around 30 feet is when it reaches the apex
        //      i mean if we increase velocity more around there it should work, not sure why it doesnt rn
        //      but it doesnt matter because 30 feet is further than the middle of the field and were never 
        //      gonna shoot from there because our robot isnt Steph Curry, and theres too many tiny variables
        //      to shoot from that far (mfw our auto aim doesnt account for general wear and tear of each note)

        // also if air resistance arises to be a problem, we can raise the magic variable until it works
        //      which is the easy way but its not technically right. The correct way is to change a_x to
        //      some accurate number (do research) then change some velocity variables to work with it
        
        //y = -5.401353307, x = -5.60307862645667 is up against subwoofer
        double a_y = -9.8; // gravity
        double v_y = 0;
        double p_y = -xDist; // vertical distance from target, this will change bc the arm will raise the further it gets away
        double a_x = -0; // pretend this doesn't exist because I don't feel like doing air resistance
        double v_x = 0;
        double p_x = -yDist; // horizontal distance from target, this will change bc the robot moves
        
        // i swear this is the only magic number and its to align the arm to 0 when against the subwoofer
        // you could go through the math to get the good number but i dont feel like it so nah
        double magic = 0.8812; // 0.8812
        double shootingvelo_y = Math.sqrt(p_y * a_y * 2) + magic;
        double shootingvelo_x = p_x/(shootingvelo_y/a_y) + magic;
        double shootingvelo = Math.sqrt(Math.pow(shootingvelo_x, 2) + Math.pow(shootingvelo_y, 2))*2;
        if(shootingvelo > 5700) {
            shootingvelo = 5700;
        }
        double rpm = (30*shootingvelo)/(Math.PI*(ShooterConstants.shooterRadius))/2;
        
        double t4 = (Math.pow(a_x, 2) + Math.pow(a_y, 2))/4;
        double t3 = (a_x*v_x + a_y*v_y);
        double t2 = (Math.pow(v_x, 2) + p_x*a_x + Math.pow(v_y, 2) + p_y*a_y - Math.pow(shootingvelo, 2));
        double t1 = 2*(p_x*v_x + p_y*v_y);
        double t0 = (Math.pow(p_x, 2) + Math.pow(p_y, 2));


        List<Double> roots = RootFinder.rootFinder(Arrays.asList(t4, t3, t2, t1, t0), 0.01);
        for(double root: roots) {
            root = Math.abs(root);
        }
        double t = -1;

        for (int i = roots.size() - 1; i >= 0; i--) {
            if (!(roots.get(i) > 0.0)) {
                roots.set(i,0.0);
            }
        }

        double minRoot = Double.MAX_VALUE;
        for (double root : roots) {
            if (root < minRoot && root > 0.0) {
                minRoot = root;
            }
        }
        if (minRoot == Double.MAX_VALUE) {
            System.out.println("no solutions");
            return null;
        }
        t = minRoot;
    



        // double maxRoot = Double.MIN_VALUE;
        // System.out.println(roots);
        // for (double root : roots) {
        //     if (root > maxRoot) {
        //         maxRoot = root;
        //     }
        // }


        // if (maxRoot == Double.MIN_VALUE) {
        //     System.out.println("no solutions");
        //     return;
        // }
        // t = maxRoot;

        double p_aimX = -(p_x + v_x*t + (a_x*(Math.pow(t, 2)))/2.0);
        double p_aimY = -(p_y + v_y*t + (a_y*(Math.pow(t, 2)))/2.0);
        double shooting_theta = Math.atan(p_aimY / p_aimX);
        double arm_theta = shooting_theta - ShooterConstants.differenceFromArm;

        return new double[]{arm_theta, rpm};
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}