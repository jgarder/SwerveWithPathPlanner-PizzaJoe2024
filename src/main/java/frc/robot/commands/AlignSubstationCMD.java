// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.IFollower;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPoint;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Constants;
import frc.robot.RobotContainer;
//import frc.robot.Constants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.DrivetrainManager;
import frc.robot.subsystems.Limelight3Subsystem;


public class AlignSubstationCMD extends Command {
  

  /** Creates a new ArmStopCMD. */
  Limelight3Subsystem limelight3Subsystem;
  CommandSwerveDrivetrain s_Swerve;

  //LL POSE X is forward and backward toward target in field space
  double k_PoseX_P = 1.20;
  double k_PoseX_I = 0.02;
  double k_PoseX_D = 0.0020;
  private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
  private final PIDController AlignXController = new PIDController(k_PoseX_P,k_PoseX_I,k_PoseX_D);

  //LL POSE Y Is left to right translation in field space
  double k_PoseY_P = 1.00;
  double k_PoseY_I = 0.02;
  double k_PoseY_D = 0.000; 
  private static final TrapezoidProfile.Constraints Z_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
  private final PIDController AlignPoseYController = new PIDController(k_PoseY_P,k_PoseY_I,k_PoseY_D);
  
  //LL pose RZ is our rotation relative to the target in field space
  double k_RZ_P = .04;//0.025;
  double k_RZ_I = 0.00;
  double k_RZ_D = 0.00;
  private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS =   new TrapezoidProfile.Constraints(8, 8);
  private final PIDController AlignRZController = new PIDController(k_RZ_P,k_RZ_I,k_RZ_D);

  
  double minXposeErrorToCorrect = .19;//.04;
  double minYposeErrorToCorrect = .08;//.04;
  double minRZErrorToCorrect = 1;//.04;

  double min_xpose_command = 0.00;
  double min_Ypose_command = 0.0;
  double min_RZ_command = 0;//0.004;

  double XP_buffer = 0;
  double YP_buffer = 0;
  double RZ_buffer = 0;

  boolean robotCentric = true;

  double XP_Setpoint = 0;
  double YP_Setpoint = 0;
  double RZ_Setpoint = 0;

  private DoubleSupplier strafeSup;

  public AlignSubstationCMD(CommandSwerveDrivetrain Thiss_Swerve, Limelight3Subsystem ThisLimelight, DoubleSupplier strafeSup) {
    s_Swerve = Thiss_Swerve;
    limelight3Subsystem = ThisLimelight;
    this.strafeSup = strafeSup;
    addRequirements(s_Swerve,limelight3Subsystem); 

  }

  Optional<Alliance> CurrentAlliance;
  double xymulti = -1.0;
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(this.limelight3Subsystem == null)   
    {
      System.out.println("LIMELIGHT IS DEAD");
      return;
    }
    AlignXController.reset();
    AlignPoseYController.reset();
    AlignRZController.reset();

    WeSeeourSubstationTag = false;
    WeSeeourCommunityTag = false;

    XP_Setpoint = 0;
    YP_Setpoint = 0;
    RZ_Setpoint = 0;
    //get our alliance red or blue
    CurrentAlliance = DriverStation.getAlliance();
    //get the target number
    int targetID = limelight3Subsystem.getTargetID();

    


    XP_buffer = 0;
    YP_buffer = 0;
    RZ_buffer = 0;

    // More complex path with holonomic rotation. Non-zero starting velocity of 2 m/s. Max velocity of 4 m/s and max accel of 3 m/s^2
  // PathPlannerTrajectory traj3 = PathPlanner.generatePath(
  // new PathConstraints(4, 3), 
  // new PathPoint(new Translation2d(1.0, 1.0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0), 0), // position, heading(direction of travel), holonomic rotation, velocity override
  // new PathPoint(new Translation2d(3.0, 3.0), Rotation2d.fromDegrees(45), Rotation2d.fromDegrees(-90)), // position, heading(direction of travel), holonomic rotation
  // new PathPoint(new Translation2d(5.0, 3.0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(-30)) // position, heading(direction of travel), holonomic rotation
  // );

  }
  int pipeline = 0;
  // Called every time the scheduler runs while the command is scheduled.
  int debounceloops = 0;
  int loopsoffbeforestopping = 50;
  //get the target number
  int targetID = -1;

  boolean WeSeeourSubstationTag = false;
  boolean WeSeeourCommunityTag = false;

  @Override
  public void execute() {
    if(this.limelight3Subsystem == null)   
    {
      System.out.println("LIMELIGHT IS DEAD");
      return;
    }
    boolean WeSeeourSubstationTagThisTime = false;
  boolean WeSeeourCommunityTagThisTime = false;
   //make sure we are in april tag pipeline before checking
   //
   //get if we have any targets
   boolean HasTarget = limelight3Subsystem.hasValidTarget();
   //get the target number
   targetID = limelight3Subsystem.getTargetID();
   //get if the closest target is for our team(ignore others obviously)
   
   double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);

   if ( (CurrentAlliance.get() == Alliance.Red) && targetID == Constants.AllianceAprilTags.Red.SpeakerCenter)//substation
   {  
     
      xymulti = -1.0;
      SetPidControlersToRedSubstation();
    //flip flag sayign we see a substation
    WeSeeourSubstationTag = true;
    WeSeeourSubstationTagThisTime = true;
   }
   if ( (CurrentAlliance.get() == Alliance.Blue) && targetID == Constants.AllianceAprilTags.Blue.SpeakerCenter)//substation
   {
    
      xymulti = 1.0;
      SetPidControlersToBlueSubstation();
    //flip flag sayign we see a substation
    WeSeeourSubstationTag = true;
    WeSeeourSubstationTagThisTime = true;
   }
   if ( (CurrentAlliance.get() == Alliance.Red) && (
    targetID == Constants.AllianceAprilTags.Red.ChainAmpSide | 
    targetID == Constants.AllianceAprilTags.Red.ChainBackSide| 
    targetID == Constants.AllianceAprilTags.Red.ChainSourceSide) )
   {
    SetPidControlersToRedCommunity();
    //flip flag sayign we see a substation
    
    xymulti = 1.0;
    WeSeeourSubstationTag = true;
    WeSeeourSubstationTagThisTime = true;
    WeSeeourCommunityTag = true;
    WeSeeourCommunityTagThisTime = true;
   }
   if ( (CurrentAlliance.get() == Alliance.Blue) && (
    targetID == Constants.AllianceAprilTags.Blue.ChainAmpSide | 
    targetID == Constants.AllianceAprilTags.Blue.ChainBackSide | 
    targetID == Constants.AllianceAprilTags.Blue.ChainSourceSide) )
   {
    xymulti = -1.0;
    SetPidControlersToBlueCommunity();
    //flip flag sayign we see a substation
    

    WeSeeourSubstationTag = true;
    WeSeeourSubstationTagThisTime = true;
    WeSeeourCommunityTag = true;
    WeSeeourCommunityTagThisTime = true;
   }
   if(!WeSeeourCommunityTagThisTime & !WeSeeourSubstationTagThisTime){
      debounceloops++;
       //if substation is at X area size then switch our speed to substation movde
       //when we are at X area set bool to true.
       if(debounceloops >= loopsoffbeforestopping)
       {
        if(false){ //RobotContainer.cowboyMode != CowboyMode.SCOREHUNTING
        //   s_Swerve.drive(
        //     new Translation2d(0,0), 
        //     0  , 
        //     !robotCentric, 
        //   true
        // ); 
        }
        else{
        //   s_Swerve.drive(
        //     new Translation2d(0,strafeVal), 
        //     0  , 
        //     !robotCentric, 
        //   true
        // ); 
        }
          
       }
      
    return;
   }
   loopsoffbeforestopping = 0;
  
   FillBuffers();
    
    //LL pose RZ is our rotation relative to the target in field space
    double RZAdjust = GetRZPoseAdjust(RZ_buffer, min_RZ_command);
    
    //LL POSE X is forward and backward toward target in field space
    double xpose_adjust = GetXPoseAdjust(XP_buffer, min_xpose_command);

    //LL POSE Y Is left to right translation in field space
    double Ypose_adjust =  GetYPoseAdjust(YP_buffer, min_Ypose_command );


    if(WeSeeourCommunityTag){
      Ypose_adjust = strafeVal;
    }

    
    double Xspeed = 1.0;//5.0 * Constants.Swerve.maxSpeed;
    double Yspeed = 1.0;//5.0 * Constants.Swerve.maxSpeed;
    double rotationspeed = 1.0;//1.0 * Constants.Swerve.maxAngularVelocity;
    
    //if we are really far away lets keep pid from going insane.
    double maxYvelocity = .5;//1.00;
    double maxXvelocity = .5;
    double maxRZvelocity = .4;//1.0;
    Ypose_adjust = MathUtil.clamp(Ypose_adjust, maxYvelocity * -1.0, maxYvelocity);
    xpose_adjust = MathUtil.clamp(xpose_adjust, maxXvelocity * -1.0, maxXvelocity);
    RZAdjust = MathUtil.clamp(RZAdjust, maxRZvelocity * -1.0, maxRZvelocity);
    // if(Math.abs(xpose_adjust) > .7){
    //   Xspeed = .25;
    // }
    // if(Math.abs(Ypose_adjust) > 1){
    //   Yspeed = .7;
    // }

    double YposeAxis = Ypose_adjust * Yspeed;
    double XposeAxis = xpose_adjust * Xspeed;
    double RZposeAxis = RZAdjust * rotationspeed;
    
    // drivetrain.applyRequest(() -> drive.withVelocityX(speedLimiterY.calculate(-joystick.getLeftY()) * MaxSpeed) // Drive forward with
    //                                                                                        // negative Y (forward)
    //         .withVelocityY(speedLimiterX.calculate(-joystick.getLeftX()) * MaxSpeed) // Drive left with negative X (left)
    //         .withRotationalRate(speedLimiterRotation.calculate(-joystick.getRightX()) * MaxAngularRate) // Drive counterclockwise with negative X (left)
    //     ).ignoringDisable(true)); 
    // s_Swerve.drive(
    //          new Translation2d(XposeAxis,YposeAxis), 
    //          RZposeAxis  , 
    //          !robotCentric, 
    //        true
    //      );

    //s_Swerve.drive(ChassisSpeeds.fromFieldRelativeSpeeds(translationAxis, strafeAxis, rotationAxis,s_Swerve.swerveOdometry.getPoseMeters().getRotation() ));
    SmartDashboard.putNumber("RZ_Current", RZ_buffer);
    SmartDashboard.putNumber("RZ_PID", RZAdjust);
    
   
    SmartDashboard.putNumber("Ypose_Current", YP_buffer);
    SmartDashboard.putNumber("Ypose_PID", Ypose_adjust);

    SmartDashboard.putNumber("Xpose", XP_buffer);
    SmartDashboard.putNumber("Xpose_PID", XposeAxis);

    SmartDashboard.putNumber("RZ_Offset", RZ_Offset);
    SmartDashboard.putNumber("Ypose_Offset", Ypose_Offset);
    SmartDashboard.putNumber("Xpose_Offset", Xpose_Offset);
  }

private void SetPidControlersToRedSubstation() {
  XP_Setpoint = -6.59;//-6.63;
  YP_Setpoint = 2.06;
  RZ_Setpoint = 180;
  //LL POSE X is forward and backward toward target in field space
  AlignXController.setSetpoint(XP_Setpoint);
  //LL POSE Y Is left to right translation in field space
  AlignPoseYController.setSetpoint(YP_Setpoint);
  //LL pose RZ is our rotation relative to the target in field space
  AlignRZController.setSetpoint(RZ_Setpoint);

  
}
private void SetPidControlersToBlueSubstation() {
  XP_Setpoint = 6.59;//6.63;
  YP_Setpoint = 3.44;
  RZ_Setpoint = 0;
  //LL POSE X is forward and backward toward target in field space
  AlignXController.setSetpoint(XP_Setpoint);
  //LL POSE Y Is left to right translation in field space
  AlignPoseYController.setSetpoint(YP_Setpoint);
  //LL pose RZ is our rotation relative to the target in field space
  AlignRZController.setSetpoint(RZ_Setpoint);
}
private void SetPidControlersToRedCommunity() {
  XP_Setpoint = 6.10;
  YP_Setpoint = -.5;
  RZ_Setpoint = 0;
  //LL POSE X is forward and backward toward target in field space
  AlignXController.setSetpoint(XP_Setpoint);
  //LL POSE Y Is left to right translation in field space
  AlignPoseYController.setSetpoint(YP_Setpoint);
  //LL pose RZ is our rotation relative to the target in field space
  AlignRZController.setSetpoint(RZ_Setpoint);

  
}
private void SetPidControlersToBlueCommunity() {
  XP_Setpoint = -6.10;
  YP_Setpoint = .5;
  RZ_Setpoint = 180;
  //LL POSE X is forward and backward toward target in field space
  AlignXController.setSetpoint(XP_Setpoint);
  //LL POSE Y Is left to right translation in field space
  AlignPoseYController.setSetpoint(YP_Setpoint);
  //LL pose RZ is our rotation relative to the target in field space
  AlignRZController.setSetpoint(RZ_Setpoint);

  
}
private void FillBuffers()
{
  double RZCurrent = limelight3Subsystem.getRZ() *-1.0;//rotation Y targetspace is ROtation Z field space?
  double xpose = limelight3Subsystem.getXPos() ;
  double Ypose = limelight3Subsystem.getYPos();
  if(Ypose != 0.00 & xpose != 0.00 & RZCurrent != 0.00) 
  {
    XP_buffer = xpose;
    YP_buffer = Ypose;
    RZ_buffer = RZCurrent;
  }
  else{debounceloops++;}
}
  //LL POSE Y Is left to right translation in field space
private double GetYPoseAdjust(double Ypose, double min_PoseY_command) {
  double ypose_adjust;
  //if(Ypose == 0){return 0;}
  //if (Math.abs(Ypose) > minYposeErrorToCorrect )
  //{
    if (Ypose < YP_Setpoint)
    {
            ypose_adjust = AlignPoseYController.calculate(Ypose) - min_PoseY_command;
    }
    else
    {
            ypose_adjust = AlignPoseYController.calculate(Ypose) + min_PoseY_command;
    }
 // }
  //else 
 // {
  //  ypose_adjust = 0;
  //}
  return ypose_adjust *xymulti;
}

  //LL POSE X is forward and backward toward target in field space
  private double GetXPoseAdjust(double fwdReverse_error, double min_Fwd_command) {
    double xpose_adjust;
    
    //if (Math.abs(fwdReverse_error) > minXposeErrorToCorrect ) //!AlignXController.atGoal()
    //{
      if (fwdReverse_error < XP_Setpoint)
      {
              xpose_adjust = AlignXController.calculate(fwdReverse_error) + min_Fwd_command;
      }
      else
      {
              xpose_adjust = AlignXController.calculate(fwdReverse_error) - min_Fwd_command;
      }
    //}
    //else 
    //{
    //  xpose_adjust = 0;
    //}
    //if(fwdReverse_error == 0){return 0;}
    return xpose_adjust *xymulti;
  }

  //LL pose RZ is our rotation relative to the target in field space
  private double GetRZPoseAdjust(double RZ, double min_spin_command) {
    double RZ_adjust = 0;
    //if(RZ == 0){return 0;}
    //double error = Math.abs(AlignRZController.getGoal().position - RZ);
    //if (Math.abs(RZ_adjust) > minRZErrorToCorrect)//!AlignRotationController.atGoal()
    //{
      if (RZ < 0)
      {
        RZ_adjust = AlignRZController.calculate(RZ*-1.0) - min_spin_command;
      }
      else
      {
        RZ_adjust = AlignRZController.calculate(RZ)*-1.0 + min_spin_command;
      }
      
      //our own tolerance script. 
      // if(Math.abs(RZ_adjust) < minRZErrorToCorrect)
      // {
      //   RZ_adjust = 0;
      // }
    //}
    //else
    //{
    //  RZ_adjust = 0;
    //}
    return RZ_adjust;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {super.end(interrupted);}

  double Xpose_Offset = 0;
  double Ypose_Offset = 0;
  double RZ_Offset = 0;
  int timesgood = 0;
  int goodneeded = 5;
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if(false){ //RobotContainer.cowboyMode == CowboyMode.SCOREHUNTING | WeSeeourCommunityTag
      return false;
    }
    if(YP_buffer != 0.00 & XP_buffer != 0.00 & RZ_buffer != 0.00) 
    {
        //SUBTRACT where we need to go, from where we are. this will give us the translations we need to make 
        Xpose_Offset = XP_buffer - XP_Setpoint;
        Ypose_Offset = YP_buffer - YP_Setpoint;
        RZ_Offset = RZ_buffer - RZ_Setpoint;

        //minXposeErrorToCorrect = .04;//.04;
        //minYposeErrorToCorrect = .02;//.04;
        //minRZErrorToCorrect = .2;//.04;
        if(Math.abs(Xpose_Offset) < minXposeErrorToCorrect && 
          Math.abs(Ypose_Offset) < minYposeErrorToCorrect  &&( 
        Math.abs(RZ_Offset) < minRZErrorToCorrect || RZ_Offset < minRZErrorToCorrect -360 ))
        {
          if(timesgood > goodneeded)
          {
            timesgood = 0;
            return true;
          }
          else
          {
            timesgood++;
            return false;
          }
          
        }
    }
    // if ( (CurrentAlliance == Alliance.Red) && targetID == Constants.AllianceAprilTags.Red.substation)
    // {  
    //   return false;
    // }
    // else if ( (CurrentAlliance == Alliance.Blue) && targetID == Constants.AllianceAprilTags.Blue.substation)
    // {
    //   return false;
    // }
    // else{
    //   return true;
    // }
      return false;
  }
}
