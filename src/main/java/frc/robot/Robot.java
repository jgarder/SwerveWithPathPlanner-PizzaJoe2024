// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.naming.spi.DirStateFactory.Result;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.LimelightHelpers.Results;
import frc.robot.RobotContainer.PizzaManager;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private final boolean UseLimelight = true;
  private final boolean UseFrontLimelight = true;
  @Override
  public void robotInit() {
    // Starts recording to data log
    DataLogManager.start();
    //enableLiveWindowInTest(true);
    m_robotContainer = new RobotContainer();

    m_robotContainer.drivetrainManager.drivetrain.getDaqThread().setThreadPriority(99);

    
  }
  double maxrotationalVelocityForLLUpdate = 100;//80;
  @Override
  public void robotPeriodic() {
   
    CommandScheduler.getInstance().run(); 
    if (UseLimelight & PizzaManager.LimelightTelemetryUpdateRequested) 
    { 
      double rotationalvelocity = m_robotContainer.drivetrainManager.drivetrain.getPigeon2().getAngularVelocityZWorld().getValueAsDouble();  
      SmartDashboard.putNumber("rotationalvelocity", rotationalvelocity);
      //Results lastResult = LimelightHelpers.getLatestResults(Constants.LimelightName).targetingResults;

      //Pose2d llPose = lastResult.getBotPose2d_wpiBlue();
      //double tl = lastResult.latency_pipeline;//LimelightHelpers.getLatency_Pipeline(Constants.LimelightName);
      //double cl = lastResult.latency_capture;//LimelightHelpers.getLatency_Capture(Constants.LimelightName);
      //double jl = lastResult.latency_jsonParse;
      // SmartDashboard.putNumber("tl", tl);
      // SmartDashboard.putNumber("cl", cl);
      // SmartDashboard.putNumber("jl", jl);
      ///
      // LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(Constants.LimelightName);
      // LimelightHelpers.PoseEstimate limelightMeasurementFront = LimelightHelpers.getBotPoseEstimate_wpiBlue(Constants.LimelightFrontName);
      // if(limelightMeasurementFront.tagCount >= 1)
      // {
      //   //m_robotContainer.drivetrainManager.drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
      //   m_robotContainer.drivetrainManager.drivetrain.addVisionMeasurement(
      //       limelightMeasurementFront.pose,
      //       limelightMeasurementFront.timestampSeconds);
      //       return;//we have a good pose estimate dont do the other pose estimate. 
      // }
      // else if(limelightMeasurement.tagCount >= 2)
      // {
      //   //m_robotContainer.drivetrainManager.drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
      //   m_robotContainer.drivetrainManager.drivetrain.addVisionMeasurement(
      //       limelightMeasurement.pose,
      //       limelightMeasurement.timestampSeconds);
      //       return;//we have a good pose estimate dont do the other pose estimate. 
      // }
      //
      // if (lastResult.valid) {
      //   m_robotContainer.drivetrainManager.drivetrain.addVisionMeasurement(llPose, Timer.getFPGATimestamp() - (tl/1000.0) - (cl/1000.0)- (jl/1000.0));
      // }
      ///////
      ///////

      ////CAMERA Front
      if(UseFrontLimelight)
      {
        boolean useMegaTag2Front = false; //set to false to use MegaTag1
        boolean doRejectUpdateFront = false;
        if(useMegaTag2Front == false)
        {
          LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(Constants.LimelightFrontName);
          
          if(mt1.tagCount == 1 && mt1.rawFiducials.length == 1)
          {
            if(mt1.rawFiducials[0].ambiguity > .7)
            {
              doRejectUpdateFront = true;
            }
            if(mt1.rawFiducials[0].distToCamera > 3)
            {
              doRejectUpdateFront = true;
            }
          }
          if(mt1.tagCount == 0)
          {
            doRejectUpdateFront = true;
          }
          if(Math.abs(rotationalvelocity) > maxrotationalVelocityForLLUpdate) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
          {
            doRejectUpdateFront = true;
          }

          if(!doRejectUpdateFront)
          {
            //m_robotContainer.drivetrainManager.drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5,9999999));
            m_robotContainer.drivetrainManager.drivetrain.addVisionMeasurement(
                mt1.pose,
                mt1.timestampSeconds);
                return;//only 1 sample per robot periodic
          }
        }
        else if (useMegaTag2Front == true)
        {
          
          LimelightHelpers.PoseEstimate mt2 = getMt2WpiBlue(Constants.LimelightFrontName);
          if(Math.abs(rotationalvelocity) > maxrotationalVelocityForLLUpdate) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
          {
            doRejectUpdateFront = true;
          }
          if(mt2.tagCount == 0)
          {
            doRejectUpdateFront = true;
          }
          if(!doRejectUpdateFront)
          {
            //m_robotContainer.drivetrainManager.drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
            m_robotContainer.drivetrainManager.drivetrain.addVisionMeasurement(
                mt2.pose,
                mt2.timestampSeconds);
                return;//only 1 sample per robot periodic
          }
        }
      }
      ///////////
      ////CAMERA UP
      boolean useMegaTag2 = false; //set to false to use MegaTag1
      boolean doRejectUpdate = false;
      if(useMegaTag2 == false)
      {
        LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(Constants.LimelightName);
        
        if(mt1.tagCount == 1 && mt1.rawFiducials.length == 1)
        {
          if(mt1.rawFiducials[0].ambiguity > .7)
          {
            doRejectUpdate = true;
          }
          if(mt1.rawFiducials[0].distToCamera > 3)
          {
            doRejectUpdate = true;
          }
        }
        if(mt1.tagCount == 0)
        {
          doRejectUpdate = true;
        }
        if(Math.abs(rotationalvelocity) > maxrotationalVelocityForLLUpdate) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
        {
          doRejectUpdate = true;
        }

        if(!doRejectUpdate)
        {
          //m_robotContainer.drivetrainManager.drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5,9999999));
          m_robotContainer.drivetrainManager.drivetrain.addVisionMeasurement(
              mt1.pose,
              mt1.timestampSeconds);
              return;//only 1 sample per robot periodic
        }
      }
      else if (useMegaTag2 == true)
      {
        

        LimelightHelpers.PoseEstimate mt2 = getMt2WpiBlue(Constants.LimelightName);
        if(Math.abs(rotationalvelocity) > maxrotationalVelocityForLLUpdate) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
        {
          doRejectUpdate = true;
        }
        if(mt2.tagCount == 0)
        {
          doRejectUpdate = true;
        }
        if(!doRejectUpdate)
        {
          //m_robotContainer.drivetrainManager.drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
          m_robotContainer.drivetrainManager.drivetrain.addVisionMeasurement(
              mt2.pose,
              mt2.timestampSeconds);
              return;//only 1 sample per robot periodic
        }
      }
      ///////////
    
      ///////
    }
  }

  private LimelightHelpers.PoseEstimate getMt2WpiBlue(String Thislimelightname) {
    Pose2d robotorientation = m_robotContainer.drivetrainManager.drivetrain.getState().Pose;
    LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(Thislimelightname);
    double degreesRotation = m_robotContainer.drivetrainManager.drivetrain.getPigeon2().getAngle();//180;//;robotorientation.getRotation().getDegrees();
    // if (mt1.tagCount >=1) {
    //   degreesRotation = mt1.pose.getRotation().getDegrees();
       LimelightHelpers.SetRobotOrientation(Thislimelightname, degreesRotation, 0, 0, 0, 0, 0);
    // }
    // if (DriverStation.getAlliance().isPresent()) {
    //   if (DriverStation.getAlliance().get() == Alliance.Red) {
    //     degreesRotation = 0;
    //   }
    //   else   degreesRotation = 180;
    // }
    
    SmartDashboard.putNumber("OrientationPidgeon2", degreesRotation);
    LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Thislimelightname);
    return mt2;
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    m_Timer.reset();
    m_Timer.start();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }
  private final Timer m_Timer = new Timer();
  @Override
  public void autonomousPeriodic() {

    if(m_Timer.get() >= 15){
      if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    }
  }

  @Override
  public void autonomousExit() {}
  int TotalTeleOpSeconds = 135;
  int SecondsNeededForTrapping = 35; //190seconds into match = 215total
  int TeleOpSecondsToPlay = TotalTeleOpSeconds - SecondsNeededForTrapping;
  private final Timer m_Timer_ClimbTime = new Timer();
  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_Timer_ClimbTime.restart();
    //PizzaManager.LimelightTelemetryUpdateRequested = false;
  
  }

  @Override
  public void teleopPeriodic() {

      if(m_Timer_ClimbTime.get() > TeleOpSecondsToPlay)
    {
        PizzaManager.RequestTrapLights = true;
        m_Timer_ClimbTime.reset();
        m_Timer_ClimbTime.stop();
    }
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
