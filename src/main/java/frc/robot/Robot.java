// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.naming.spi.DirStateFactory.Result;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.LimelightHelpers.Results;
import frc.robot.RobotContainer.PizzaManager;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private final boolean UseLimelight = true;

  @Override
  public void robotInit() {
    //enableLiveWindowInTest(true);
    m_robotContainer = new RobotContainer();

    m_robotContainer.drivetrainManager.drivetrain.getDaqThread().setThreadPriority(99);
  
  }
  @Override
  public void robotPeriodic() {
   
    CommandScheduler.getInstance().run(); 
    if (UseLimelight & PizzaManager.LimelightTelemetryUpdateRequested) {    
      Results lastResult = LimelightHelpers.getLatestResults(Constants.LimelightName).targetingResults;

      Pose2d llPose = lastResult.getBotPose2d_wpiBlue();
      double tl = LimelightHelpers.getLatency_Pipeline(Constants.LimelightName);
      double cl = LimelightHelpers.getLatency_Capture(Constants.LimelightName);
      if (lastResult.valid) {
        m_robotContainer.drivetrainManager.drivetrain.addVisionMeasurement(llPose, Timer.getFPGATimestamp() - (tl/1000.0) - (cl/1000.0));
      }
    }
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
