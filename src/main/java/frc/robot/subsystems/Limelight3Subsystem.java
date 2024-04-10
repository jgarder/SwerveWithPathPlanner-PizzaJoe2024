// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
//import frc.robot.FieldConstants;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Optional;

import edu.wpi.first.networktables.NetworkTable;

public class Limelight3Subsystem extends SubsystemBase {

    private NetworkTable limelight;//Table for the limelight


    //get our alliance red or blue
    Optional<Alliance> CurrentAlliance = DriverStation.getAlliance();




  /** Creates a new Limelight3Subsystem. */
  public Limelight3Subsystem() {


  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    getdataToDashboard();

   if(!CurrentAlliance.isPresent()){CurrentAlliance = DriverStation.getAlliance();}
  
   if(!CurrentAlliance.isPresent()){System.out.println("no alliance!!"); return;}
    
  }

  
  public NetworkTable getdataToDashboard()
  {
    //NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    //latestInfo = table;
    var wpiblue = LimelightHelpers.getBotPose_wpiBlue(Constants.LimelightName);
    if(wpiblue != null && wpiblue.length >= 6)
    {
    SmartDashboard.putNumber("PoseWPIBLueX", wpiblue[0]);
    SmartDashboard.putNumber("PoseWPIBLuey", wpiblue[1]);
    SmartDashboard.putNumber("PoseWPIBLueRoll", wpiblue[5]);
    }

    var wpiblue2 = LimelightHelpers.getBotPose_wpiBlue(Constants.LimelightFrontName);
    if(wpiblue2 != null && wpiblue2.length >= 6)
    {
    SmartDashboard.putNumber("FrontPoseWPIBLueX", wpiblue2[0]);
    SmartDashboard.putNumber("FrontPoseWPIBLuey", wpiblue2[1]);
    SmartDashboard.putNumber("FrontPoseWPIBLueRoll", wpiblue2[5]);
    }

    //SmartDashboard.putNumber("PoseWPIBLueLatency", botpose_wpiblueNTE.getDoubleArray(new double[6])[6]);
    //System.out.println("");
    //post to smart dashboard periodically
    //SmartDashboard.putNumber("LimelightX", LimelightHelpers.getTX(Constants.LimelightName));
    //SmartDashboard.putNumber("LimelightY", LimelightHelpers.getTY(Constants.LimelightName));
    //SmartDashboard.putNumber("LimelightArea", LimelightHelpers.getTA(Constants.LimelightName));
    
    int roundingpower = 1000;
    //SmartDashboard.putNumber("LL Distance", Math.floor(LimelightHelpers.getdistance()*roundingpower)/roundingpower);
    //SmartDashboard.putNumber("LL Pose X", Math.floor(getXPos()*roundingpower)/roundingpower);
    //SmartDashboard.putNumber("LL Pose Y", Math.floor(getYPos()*roundingpower)/roundingpower);
    //SmartDashboard.putNumber("LL Pose Z", Math.floor(getZPos()*roundingpower)/roundingpower);
    //SmartDashboard.putNumber("LL Pitch", Math.floor(getPitch()*roundingpower)/roundingpower);
    //SmartDashboard.putNumber("LL Pose Yaw", Math.floor(getYaw()*roundingpower)/roundingpower);
    //SmartDashboard.putNumber("LL Roll", Math.floor(getRoll()*roundingpower)/roundingpower);
    SmartDashboard.putNumber("LL TargetID", LimelightHelpers.getFiducialID(Constants.LimelightName));
    return limelight;
  }




	



	

	




	



}

