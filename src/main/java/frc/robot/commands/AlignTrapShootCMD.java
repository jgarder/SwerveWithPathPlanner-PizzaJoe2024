//Team 8608 alignment. 
package frc.robot.commands;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.DrivetrainManager;
import frc.robot.subsystems.Limelight3Subsystem;


public class AlignTrapShootCMD extends Command {
  
  DrivetrainManager drivetrainManager;
  Optional<Alliance> CurrentAlliance;
  private DoubleSupplier strafeSup;

  private final PIDController AlignXController = new PIDController(Constants.ChassisPid.k_PoseX_P,Constants.ChassisPid.k_PoseX_I,Constants.ChassisPid.k_PoseX_D);
  private final PIDController AlignPoseYController = new PIDController(Constants.ChassisPid.k_PoseY_P,Constants.ChassisPid.k_PoseY_I,Constants.ChassisPid.k_PoseY_D);
  private final PIDController AlignRZController = new PIDController(Constants.ChassisPid.k_RZ_P,Constants.ChassisPid.k_RZ_I,Constants.ChassisPid.k_RZ_D);
  
  double minXposeErrorToCorrect = Constants.ChassisPid.minXposeErrorToCorrect/2; //.03175 Meters error is 1.25"
  double minYposeErrorToCorrect = Constants.ChassisPid.minYposeErrorToCorrect/2;
  double minRZErrorToCorrect = Constants.ChassisPid.minRZErrorToCorrect/2;

  double min_xpose_command = Constants.ChassisPid.min_xpose_command;
  double min_Ypose_command = Constants.ChassisPid.min_Ypose_command;
  double min_RZ_command = Constants.ChassisPid.min_RZ_command;

  //if we are really far away lets keep pid from going insane.
  double maxYvelocity = Constants.ChassisPid.maxYvelocity;
  double maxXvelocity = Constants.ChassisPid.maxXvelocity;
  double maxRZvelocity = Constants.ChassisPid.maxRZvelocity;

  Pose2d CurrentPose;//this is our latest position according to our chassis odometry

  Pose2d TargetPose;//this is where we wnt to go in field space coords X,y,Rotation

  Pose2d PoseOffset;//This is how far we are from where we want to be. this is CurrentPose minus TargetPose.

  //these are used as multipliers for inverting axis (if needed for a given season)
  double Xspeed = 1.0;
  double Yspeed = 1.0;
  double rotationspeed = 1.0;

  //how many times must robot be in location for this to command to finish.
  int timesgood = 0;
  int goodneeded = 5;

  public String Alignxyname = "AlignXY";
  public String AlignRotname = "AlignRot";

  //////////////////////////////////////////////////////////// SETUP TARGET POSITIONS////////////////////////////////////////////
  //This is a top function because this is the logic for deciding what points are to be chosen and why. 
  private void SetupTargetPosition() {
  double targetID = LimelightHelpers.getFiducialID(Constants.LimelightName);
  if ( (CurrentAlliance.get() == Alliance.Red) )//substation
  { 
    TargetPose = GetLatestPoseToBuffer();//default incase a tag is not seen. 
    if (targetID == Constants.AllianceAprilTags.Red.StageAmpSide) 
    {
      TargetPose = Constants.TargetLocations.Red.TrapFloorStageAmpSide;
    } 
    else if (targetID == Constants.AllianceAprilTags.Red.StageSourceSide) 
    {
      TargetPose = Constants.TargetLocations.Red.TrapFloorStageSourceSide;
    } 
    else if (targetID == Constants.AllianceAprilTags.Red.StageCenterSide) 
    {
      TargetPose = Constants.TargetLocations.Red.TrapFloorStageCenterSide;
    }
    else end(true);


    Xspeed = Constants.TargetLocations.Red.Xspeed;
    Yspeed = Constants.TargetLocations.Red.Yspeed;
    rotationspeed = Constants.TargetLocations.Red.rotationspeed;
  }
  else if ( (CurrentAlliance.get() == Alliance.Blue))//substation
  {
    TargetPose = GetLatestPoseToBuffer();//default incase a tag is not seen. hack fix should just end?
    if (targetID == Constants.AllianceAprilTags.Blue.StageAmpSide) 
    {
      TargetPose = Constants.TargetLocations.Blue.TrapFloorStageAmpSide;
    } 
    else if (targetID == Constants.AllianceAprilTags.Blue.StageSourceSide) 
    {
      TargetPose = Constants.TargetLocations.Blue.TrapFloorSourceSide;
    } 
    else if (targetID == Constants.AllianceAprilTags.Blue.StageCenterSide) 
    {
      TargetPose = Constants.TargetLocations.Blue.TrapFloorStageCenterSide;
    } 
    else end(true);
    Xspeed = Constants.TargetLocations.Blue.Xspeed;
    Yspeed = Constants.TargetLocations.Blue.Yspeed;
    rotationspeed = Constants.TargetLocations.Blue.rotationspeed;

  }

    //setup Pids to Want to Goto our Target position.
    AlignXController.setSetpoint(TargetPose.getX());
    AlignPoseYController.setSetpoint(TargetPose.getY());
    AlignRZController.setSetpoint(0); // we feed an offset to our controller and attempt to get to 0; HACK FIX?.
  }
  /////////////////////////////////////////////END TARGET SETUP 

  //this is the constructor, this is whats called when the object is built
  public AlignTrapShootCMD(DrivetrainManager Thiss_Swerve,DoubleSupplier strafeSup) {
    drivetrainManager = Thiss_Swerve;
    this.strafeSup = strafeSup;
    addRequirements(drivetrainManager); 
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //get our alliance red or blue.
    CurrentAlliance = DriverStation.getAlliance();
    //setup target location based on current alliance, we *should* only need to do this once on intialization
    SetupTargetPosition();

    //DEBUGGING STUFF CAN BE TURNED OFF FOR COMPETITION
    PidTune(Alignxyname);
    SmartDashboard.putNumber(Alignxyname + " P Gain", AlignXController.getP());
    SmartDashboard.putNumber(Alignxyname + " I Gain", AlignXController.getI());
    SmartDashboard.putNumber(Alignxyname + " D Gain", AlignXController.getD());
    PidTuneRot(AlignRotname);
    SmartDashboard.putNumber(AlignRotname + " P Gain", AlignRZController.getP());
    SmartDashboard.putNumber(AlignRotname + " I Gain", AlignRZController.getI());
    SmartDashboard.putNumber(AlignRotname + " D Gain", AlignRZController.getD());
    /////////////
  }

  
  

  @Override
  public void execute() {
    //we cant align to an alliance tag if the DS has no alliance. 
    if(!CurrentAlliance.isPresent()){return;}
    
    //get latest pose from odometry (which is updated by limelight elsewhere)
    GetLatestPoseToBuffer();
    //GetLatestPoseToBuffer();
    UpdateOffsetsFromTarget();

    //Add min command to keep things moving. 
    double RZAdjust = AlignRZController.calculate(PoseOffset.getRotation().getDegrees());//GetRZPoseAdjust(RZCurrent2d, min_RZ_command);
    double xpose_adjust = AlignXController.calculate(CurrentPose.getX());//GetXPoseAdjust(XP_buffer, min_xpose_command);
    double Ypose_adjust = AlignPoseYController.calculate(CurrentPose.getY());//GetYPoseAdjust(YP_buffer, min_Ypose_command );
    
    //Add in (or subtract extra) minimum command;
    RZAdjust += (Math.signum(RZAdjust)*min_RZ_command);
    xpose_adjust += (Math.signum(xpose_adjust)*min_xpose_command);
    Ypose_adjust += (Math.signum(Ypose_adjust)*min_Ypose_command);
    
    //if user wants to strafe let that in
    double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
    if(strafeVal > minRZErrorToCorrect){
      RZAdjust = strafeVal;
    }
       
    //clamp all results to a max (and negative max) top speed
    Ypose_adjust = MathUtil.clamp(Ypose_adjust, -maxYvelocity, maxYvelocity);
    xpose_adjust = MathUtil.clamp(xpose_adjust, -maxXvelocity, maxXvelocity);
    RZAdjust = MathUtil.clamp(RZAdjust, -maxRZvelocity, maxRZvelocity);

    //invert any axis that needs inverting this year
    double YposeAxis = Ypose_adjust * Yspeed;
    double XposeAxis = xpose_adjust * Xspeed;
    double RZposeAxis = RZAdjust * rotationspeed;

    //JITTER REMOVER, enable if you have bad pids or loose tolerances
    //if any axis is within tolerance then stop jittering.
    //  if(isRotInTarget()) {RZposeAxis = 0;}
    //  if(IsXInTarget()){ xpose_adjust = 0;}
    //  if(IsYInTarget()) { Ypose_adjust = 0;}
    //////////////////////////////////////////////////

    //Drive the swerve drive with whatever Axis adjustments are needed.
    drivetrainManager.MoveRobotToTargetInFieldCoordinates(YposeAxis, XposeAxis, RZposeAxis);

    //show some data
    SmartDashboard.putBoolean("isRotInTarget", isRotInTarget());
    SmartDashboard.putBoolean("IsYInTarget", IsYInTarget());
    SmartDashboard.putBoolean("IsXInTarget", IsXInTarget());
    //pid tune if you want for debugging, not needed in competition. 
    PidTune(Alignxyname);
    PidTuneRot(AlignRotname);
  }




  
// Called once the command ends or is interrupted.
@Override
public void end(boolean interrupted) {super.end(interrupted);}


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if(CurrentPose != null) 
    {
        if(IsXInTarget() && IsYInTarget()  && isRotInTarget())
        {
          if(timesgood > goodneeded)
          {
            //timesgood = 0;
            //Stop movement if we are there.
            drivetrainManager.StopDriveTrain();
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

  //get the offset of where we need to go.
  private void UpdateOffsetsFromTarget() {
    //SUBTRACT where we need to go, from where we are. this will give us the translations we need to make 
    double Xpose_Offset = CurrentPose.getX() - TargetPose.getX();
    double Ypose_Offset = CurrentPose.getY() - TargetPose.getY();             
    Rotation2d RZ_Offset2 = CurrentPose.getRotation().minus(TargetPose.getRotation());
    PoseOffset = new Pose2d(Xpose_Offset, Ypose_Offset, RZ_Offset2);
    return;
  }
  //grabs Drivetrain Information and gives us a local version to work with.
  private Pose2d GetLatestPoseToBuffer()
  {
    CurrentPose = drivetrainManager.drivetrain.getState().Pose;
    return drivetrainManager.drivetrain.getState().Pose;
  }

  private boolean IsXInTarget() {
    return Math.abs(PoseOffset.getX()) < minXposeErrorToCorrect;
  }

  private boolean IsYInTarget() {
    return Math.abs(PoseOffset.getY()) < minYposeErrorToCorrect;
  }

  private boolean isRotInTarget() {
    return Math.abs(PoseOffset.getRotation().getDegrees()) < minRZErrorToCorrect;
  }

  //This is used with the SmartDashboard to Tune the PID. Unneeded for competition.
  private void PidTune(String PidName) {
    double p = SmartDashboard.getNumber(PidName + " P Gain", Constants.ChassisPid.k_PoseX_P);
    double i = SmartDashboard.getNumber(PidName + " I Gain", Constants.ChassisPid.k_PoseX_I);
    double d = SmartDashboard.getNumber(PidName + " D Gain", Constants.ChassisPid.k_PoseX_D);
      
    if((p != AlignXController.getP())) { AlignXController.setP(p); }
    if((i != AlignXController.getI())) { AlignXController.setI(i); }
    if((d != AlignXController.getD())) { AlignXController.setD(d); }

    if((p != AlignPoseYController.getP())) { AlignPoseYController.setP(p); }
    if((i != AlignPoseYController.getI())) { AlignPoseYController.setI(i); }
    if((d != AlignPoseYController.getD())) { AlignPoseYController.setD(d); }
  }
  private void PidTuneRot(String PidName) {
    double p = SmartDashboard.getNumber(PidName + " P Gain", Constants.ChassisPid.k_RZ_P);
    double i = SmartDashboard.getNumber(PidName + " I Gain", Constants.ChassisPid.k_RZ_I);
    double d = SmartDashboard.getNumber(PidName + " D Gain", Constants.ChassisPid.k_RZ_D);
      
    if((p != AlignRZController.getP())) { AlignRZController.setP(p); }
    if((i != AlignRZController.getI())) { AlignRZController.setI(i); }
    if((d != AlignRZController.getD())) { AlignRZController.setD(d); }

  }
}
