// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainManager;


public class AlignSpeakerCMD extends Command {
  

  /** Creates a new ArmStopCMD. */
  DrivetrainManager drivetrainManager;
  Optional<Alliance> CurrentAlliance;
  private DoubleSupplier strafeSup;
  public final SwerveRequest.RobotCentric RobotCentricdrive;

  private final PIDController AlignXController = new PIDController(Constants.ChassisPid.k_PoseX_P,Constants.ChassisPid.k_PoseX_I,Constants.ChassisPid.k_PoseX_D);
  private final PIDController AlignPoseYController = new PIDController(Constants.ChassisPid.k_PoseY_P,Constants.ChassisPid.k_PoseY_I,Constants.ChassisPid.k_PoseY_D);
  private final PIDController AlignRZController = new PIDController(Constants.ChassisPid.k_RZ_P,Constants.ChassisPid.k_RZ_I,Constants.ChassisPid.k_RZ_D);
  
  double minXposeErrorToCorrect = Constants.ChassisPid.minXposeErrorToCorrect;
  double minYposeErrorToCorrect = Constants.ChassisPid.minYposeErrorToCorrect;
  double minRZErrorToCorrect = Constants.ChassisPid.minRZErrorToCorrect;

  double min_xpose_command = Constants.ChassisPid.min_xpose_command;
  double min_Ypose_command = Constants.ChassisPid.min_Ypose_command;
  double min_RZ_command = Constants.ChassisPid.min_RZ_command;

  //if we are really far away lets keep pid from going insane.
  double maxYvelocity = Constants.ChassisPid.maxYvelocity;
  double maxXvelocity = Constants.ChassisPid.maxXvelocity;
  double maxRZvelocity = Constants.ChassisPid.maxRZvelocity;

  double XP_buffer = 0;
  double YP_buffer = 0;
  //double RZ_buffer = 0;
  Rotation2d RZCurrent2d = new Rotation2d();
  
  double XP_Setpoint = 0;
  double YP_Setpoint = 0;
  double RZ_Setpoint = 0;
  Rotation2d RzTarget = new Rotation2d();

  double Xspeed = 1.0;
  double Yspeed = 1.0;
  double rotationspeed = 1.0;



  public AlignSpeakerCMD(DrivetrainManager Thiss_Swerve, DoubleSupplier strafeSup) {
    drivetrainManager = Thiss_Swerve;
    this.strafeSup = strafeSup;
    RobotCentricdrive = Thiss_Swerve.RobotCentricdrive;
    addRequirements(drivetrainManager); 
  }

  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //get our alliance red or blue
    CurrentAlliance = DriverStation.getAlliance();
  }


  @Override
  public void execute() {

    if(!CurrentAlliance.isPresent()){return;}
    
    SetupTargetPosition();
    

    GetLatestPoseToBuffer();
    
    //Add min command to keep things moving. 
    double RZAdjust = GetRZPoseAdjust(RZCurrent2d, min_RZ_command);
    double xpose_adjust = GetXPoseAdjust(XP_buffer, min_xpose_command);
    double Ypose_adjust =  GetYPoseAdjust(YP_buffer, min_Ypose_command );

    double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
    if(strafeVal > minRZErrorToCorrect){
      RZAdjust = strafeVal;
    }
       

    Ypose_adjust = MathUtil.clamp(Ypose_adjust, -maxYvelocity, maxYvelocity);
    xpose_adjust = MathUtil.clamp(xpose_adjust, -maxXvelocity, maxXvelocity);
    RZAdjust = MathUtil.clamp(RZAdjust, -maxRZvelocity, maxRZvelocity);

    double YposeAxis = Ypose_adjust * Yspeed;
    double XposeAxis = xpose_adjust * Xspeed;
    double RZposeAxis = RZAdjust * rotationspeed;
    if(isRotInTarget()) {RZposeAxis = 0;}
    if(IsXInTarget()){ xpose_adjust = 0;}
    if(IsYInTarget()) { Ypose_adjust = 0;}
    
        drivetrainManager.drivetrain.setControl(drivetrainManager.FCdrive.withVelocityX(XposeAxis * drivetrainManager.MaxSpeed) // Drive forward with // negative Y (forward)
            .withVelocityY(YposeAxis * drivetrainManager.MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(RZposeAxis * drivetrainManager.MaxAngularRate) // Drive counterclockwise with negative X (left)
        );

    SmartDashboard.putNumber("R_Curr", RZCurrent2d.getDegrees());
    SmartDashboard.putNumber("R_PID", RZposeAxis);
    
   
    SmartDashboard.putNumber("Y_Curr", YP_buffer);
    SmartDashboard.putNumber("Y_PID", YposeAxis);

    SmartDashboard.putNumber("X_Curr", XP_buffer);
    SmartDashboard.putNumber("X_PID", XposeAxis);

    //SmartDashboard.putNumber("RZ_Offset", RZ_Offset);
    SmartDashboard.putNumber("Ypose_Offset", Ypose_Offset);
    SmartDashboard.putNumber("Xpose_Offset", Xpose_Offset);
    SmartDashboard.putNumber("RZ_Offset", RZ_Offset2.getDegrees());

    SmartDashboard.putBoolean("isRotInTarget", isRotInTarget());
    SmartDashboard.putBoolean("IsYInTarget", IsYInTarget());
    SmartDashboard.putBoolean("IsXInTarget", IsXInTarget());
  }


  private void SetupTargetPosition() {
    if ( (CurrentAlliance.get() == Alliance.Red) )//substation
    {  
      XP_Setpoint = Constants.TargetLocations.Red.SpeakerCenter_XP_Setpoint;
      YP_Setpoint = Constants.TargetLocations.Red.SpeakerCenter_YP_Setpoint;
      RZ_Setpoint = Constants.TargetLocations.Red.SpeakerCenter_RZ_Setpoint;
      

      Xspeed = Constants.TargetLocations.Red.Xspeed;
      Yspeed = Constants.TargetLocations.Red.Yspeed;
      rotationspeed = Constants.TargetLocations.Red.rotationspeed;
    }
    else if ( (CurrentAlliance.get() == Alliance.Blue))//substation
    {
      XP_Setpoint = Constants.TargetLocations.Blue.SpeakerCenter_XP_Setpoint;
      YP_Setpoint = Constants.TargetLocations.Blue.SpeakerCenter_YP_Setpoint;
      RZ_Setpoint = Constants.TargetLocations.Blue.SpeakerCenter_RZ_Setpoint;

      Xspeed = Constants.TargetLocations.Blue.Xspeed;
      Yspeed = Constants.TargetLocations.Blue.Yspeed;
      rotationspeed = Constants.TargetLocations.Blue.rotationspeed;

    }
    //setup target
    RzTarget = Rotation2d.fromDegrees(RZ_Setpoint);
    //LL POSE X is forward and backward toward target in field space
    AlignXController.setSetpoint(XP_Setpoint);
    //LL POSE Y Is left to right translation in field space
    AlignPoseYController.setSetpoint(YP_Setpoint);
    //LL pose RZ is our rotation relative to the target in field space
    AlignRZController.setSetpoint(0); // we feed an offset to our controller and attempt to get to 0; HACK FIX?.
  }

private void GetLatestPoseToBuffer()
{
  RZCurrent2d = drivetrainManager.drivetrain.getState().Pose.getRotation();//limelight3Subsystem.getRZPosWpiBlue();//rotation Y targetspace is ROtation Z field space?
  //double RZCurrent = drivetrainManager.drivetrain.getState().Pose.getRotation().getDegrees();//limelight3Subsystem.getRZPosWpiBlue();//rotation Y targetspace is ROtation Z field space?
  double xpose = drivetrainManager.drivetrain.getState().Pose.getX();//limelight3Subsystem.getXPosWpiBlue();
  double Ypose = drivetrainManager.drivetrain.getState().Pose.getY();//limelight3Subsystem.getYPosWpiBlue();
  if(Ypose != 0.00 & xpose != 0.00 & RZCurrent2d != null) 
  {
    XP_buffer = xpose;
    YP_buffer = Ypose;
    //RZ_buffer = RZCurrent;
  }
  else{}
}
  //LL POSE Y Is left to right translation in field space
private double GetYPoseAdjust(double Ypose, double min_PoseY_command) {
  double ypose_adjust;
    if (Ypose < YP_Setpoint)
    {
            ypose_adjust = AlignPoseYController.calculate(Ypose) + min_PoseY_command;
    }
    else
    {
            ypose_adjust = AlignPoseYController.calculate(Ypose) - min_PoseY_command;
    }
  return ypose_adjust ;
}

  //LL POSE X is forward and backward toward target in field space
  private double GetXPoseAdjust(double Xpose, double min_Fwd_command) {
    double xpose_adjust;
      if (Xpose < XP_Setpoint)
      {
              xpose_adjust = AlignXController.calculate(Xpose) + min_Fwd_command;
      }
      else
      {
              xpose_adjust = AlignXController.calculate(Xpose) - min_Fwd_command;
      }
    return xpose_adjust ;
  }

  //LL pose RZ is our rotation relative to the target in field space
  private double GetRZPoseAdjust(Rotation2d RZ, double min_spin_command) {
    var RZ_offset_FromTarget = RZ.minus(RzTarget);
    double RZ_adjust = 0;
      if (RZ_offset_FromTarget.getDegrees() < 0)
      {
        RZ_adjust = AlignRZController.calculate(RZ_offset_FromTarget.getDegrees()) + min_spin_command;
      }
      else
      {
        RZ_adjust = AlignRZController.calculate(RZ_offset_FromTarget.getDegrees()) - min_spin_command;
      }
      
    return RZ_adjust;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {super.end(interrupted);}

  double Xpose_Offset = 0;
  double Ypose_Offset = 0;
  //double RZ_Offset = 0;
  Rotation2d RZ_Offset2 = new Rotation2d();
  int timesgood = 0;
  int goodneeded = 5;
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if(YP_buffer != 0.00 & XP_buffer != 0.00 & RZCurrent2d != null) 
    {
        //SUBTRACT where we need to go, from where we are. this will give us the translations we need to make 
        Xpose_Offset = XP_buffer - XP_Setpoint;
        Ypose_Offset = YP_buffer - YP_Setpoint;
        
          //RZ_Offset = RZ_buffer - RZ_Setpoint;
          
        RZ_Offset2 = RZCurrent2d.minus(RzTarget);
        

        if(IsXInTarget() && IsYInTarget()  && isRotInTarget())
        {
          if(timesgood > goodneeded)
          {
            timesgood = 0;
            //Stop movement if we are there.
            drivetrainManager.drivetrain.setControl(RobotCentricdrive.withVelocityX(0 * drivetrainManager.MaxSpeed) // Drive forward with // negative Y (forward)
            .withVelocityY(0 * drivetrainManager.MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(0 * drivetrainManager.MaxAngularRate) // Drive counterclockwise with negative X (left)
            );
            //
            return true;
          }
          else
          {
            timesgood++;
            return false;
          }
          
        }
    }
      return false;
  }

  private boolean IsXInTarget() {
    return Math.abs(Xpose_Offset) < minXposeErrorToCorrect;
  }

  private boolean IsYInTarget() {
    return Math.abs(Ypose_Offset) < minYposeErrorToCorrect;
  }

  private boolean isRotInTarget() {
    return Math.abs(RZ_Offset2.getDegrees()) < minRZErrorToCorrect ;//|| RZ_Offset < minRZErrorToCorrect -180;
  }
}
