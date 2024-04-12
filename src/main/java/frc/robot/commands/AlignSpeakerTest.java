package frc.robot.commands;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CANdleSystem;
import frc.robot.subsystems.DeliveryShooter;
import frc.robot.subsystems.DeliveryTilt;
import frc.robot.subsystems.DrivetrainManager;

public class AlignSpeakerTest extends Command {
      

  double kP = 0.012;//0.017;
  double kI = .0000000001;
  double kD = 0.002;

  DrivetrainManager drivetrainManager;
  CANdleSystem m_candleSubsystem;
  DeliveryTilt DTilt;
  DeliveryShooter Dshooter;
  double kMaxAngularSpeed = 2;
  double kMaxSpeed = 5;


  DoubleSupplier XAxis;
  DoubleSupplier YAxis;

  //double rotationOffset = 0;
  double totaltilt=0;
  double TotalRpm=0;
  double currentPercentOfMaxDistance = 0;

  final double minDistanceToShootFrom = 1.41;//1.20;
  final double maxDistToShootFrom = 5.0;//23.2;
  final double MidDistance = .5;

  public static final double kMaxRotationOffset = 1.0;//.5//2.0;//1.43;
  public static final double kMetersOffCenterForCorrection = 1.25;
  public static final double kinchesToAddToAimCorrection = 24;
  public static final double kRpmtolerance = 4.0;

  double MaxRotationOffset = kMaxRotationOffset;//.5//2.0;//1.43;
  double MetersOffCenterForCorrection = kMetersOffCenterForCorrection;
  double inchesToAddToAimCorrection = kinchesToAddToAimCorrection;



  double TiltAtShortestDistance = 0;//Constants.DeliveryHead.Tilt_Position_Speaker_Closest;
  double TiltAtMid = 0;
  double TiltAtMaxDistance = 0;//Constants.DeliveryHead.Tilt_Position_Speaker_Furthest;//Tilt at max distance (podium)

  final double RPMatShortestDistance = Constants.DeliveryHead.ShooterRpmSpeakerClose;
  final double RPMAtMaxDistance = Constants.DeliveryHead.ShooterRpmSpeakerPodium;//RPM at max distance (podium)
  

  double RpmAddPossible = RPMAtMaxDistance-RPMatShortestDistance;
  double TiltAddPossible = TiltAtMaxDistance-TiltAtShortestDistance;
  private final PIDController AlignRZController = new PIDController(kP,kI,kD);
  Optional<Alliance> CurrentAlliance;
  public String AlignRotname = "AlignRotShot";
  double RPMpercentageTolerance = kRpmtolerance;
  int timesgood = 0;
  int goodneeded = 0;

  boolean RotationInRange = false;
  boolean RPMReadyTofire = false;

  private final Timer m_SettleTimer = new Timer();
  double SettleTimeAtCorrectRPM = .15;
  double min_RZ_command = Constants.ChassisPid.min_RZ_command;

private final Timer m_TimerToShoot = new Timer();
  public AlignSpeakerTest(DrivetrainManager dtm, DeliveryTilt dTilt,DeliveryShooter dshooter,CANdleSystem candleSubsystem,DoubleSupplier Xaxis, DoubleSupplier Yaxis){
    m_candleSubsystem = candleSubsystem;
    DTilt = dTilt;
    Dshooter = dshooter;
    XAxis = Xaxis;
    YAxis =  Yaxis;
    drivetrainManager=dtm;
    SmartDashboard.putNumber("Passing-RPM", Constants.DeliveryHead.PassingRPM);
        //addRequirements(m_DeliveryHolder);
    }
    
  Pose2d CurrentPose;//this is our latest position according to our chassis odometry

  Pose2d TargetPose;//this is where we wnt to go in field space coords X,y,Rotation

  Pose2d PoseOffset;//This is how far we are from where we want to be. this is CurrentPose minus TargetPose.
  
  double AlignRzSetpoint = 0;//-7.0;//-1;//-13 means we always look to the right of where we want to shoot.

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    tid = 0;
    //tid = LimelightHelpers.getFiducialID("limelight");
    CurrentPose = drivetrainManager.drivetrain.getState().Pose;
    //get our alliance red or blue.
   CurrentAlliance = DriverStation.getAlliance();
   AlignRZController.setSetpoint(AlignRzSetpoint);  

    TiltAtShortestDistance = Constants.DeliveryHead.Tilt_Position_Speaker_Closest;
    TiltAtMid = Constants.DeliveryHead.Tilt_Position_Speaker_Mid;
    TiltAtMaxDistance = Constants.DeliveryHead.Tilt_Position_Speaker_Furthest;//Tilt at max distance (podium)
    Dshooter.resetSettleTimer();
    //System.out.println("Closest num" + Constants.DeliveryHead.Tilt_Position_Speaker_Closest);
    /////////////////
    PidTuneRot(AlignRotname);
    SmartDashboard.putNumber(AlignRotname + " P Gain", AlignRZController.getP());
    SmartDashboard.putNumber(AlignRotname + " I Gain", AlignRZController.getI());
    SmartDashboard.putNumber(AlignRotname + " D Gain", AlignRZController.getD());
    ///////////////
    DTilt.resetSettleTimer();
    m_TimerToShoot.restart();
    SetTarget();
    
  }
   double tid = 0;
   double tidfront = 0;
   double passingrotationMulti = 2.0;
  @Override
  public void execute() {
    //
    double latestRpm = SmartDashboard.getNumber("Passing-RPM", Constants.DeliveryHead.PassingRPM);
    if(Constants.DeliveryHead.PassingRPM != latestRpm){Constants.DeliveryHead.PassingRPM = latestRpm;}
    ///
    tid = LimelightHelpers.getFiducialID(Constants.LimelightName);
    tidfront = LimelightHelpers.getFiducialID(Constants.LimelightFrontName);
    //update current pose
    CurrentPose = drivetrainManager.drivetrain.getState().Pose;
    ///
    //////check alliance and set target
    if(!CurrentAlliance.isPresent()){return;}
    SetTarget();
    // get offsets of current pose from target pose
    UpdateOffsetsFromTarget();
    
    //
    //distance check
    ///
    var Currdistance = Math.abs(Math.hypot(PoseOffset.getX(), PoseOffset.getY()));//Field centric is X is the away distance, robot centric while pointing at the spot Y is our distance.    //LimelightHelpers.getTY("limelight");
    currentPercentOfMaxDistance = Currdistance/(maxDistToShootFrom);
    var currentPercentOfMidDistance = (Math.abs(Currdistance - minDistanceToShootFrom)/(maxDistToShootFrom*MidDistance))*2;
    var currentPercentOfmidToMaxDistance = Math.abs(Currdistance -maxDistToShootFrom*MidDistance)/(maxDistToShootFrom -maxDistToShootFrom*MidDistance);
    SmartDashboard.putNumber("currentPercentOfMidDistance", currentPercentOfMidDistance);
    SmartDashboard.putNumber("currentPercentOfmidToMaxDistance", currentPercentOfmidToMaxDistance);
    SmartDashboard.putNumber("ShotDistanceMeters", Currdistance);
    var MaxDistMultiplier = currentPercentOfMaxDistance;//MathUtil.clamp(currentPercentOfMaxDistance, 0.000, 1.000);
    SmartDashboard.putNumber("MaxDistMultiplier", MaxDistMultiplier);
    ///
    double RpmAdded = RpmAddPossible* MaxDistMultiplier;
    TotalRpm = RpmAdded+RPMatShortestDistance;
    /////
    
    //
    if(currentPercentOfMaxDistance < MidDistance)
    {
      var currDistmulti = currentPercentOfMidDistance;//Math.abs(Currdistance - minDistanceToShootFrom)/(maxDistToShootFrom- minDistanceToShootFrom);// Math.abs(Currdistance)/(maxDistToShootFrom*MidDistance);
      SmartDashboard.putNumber("currDistmulti", currDistmulti);
      TiltAddPossible = TiltAtMid-TiltAtShortestDistance;
      double TiltAdded = TiltAddPossible* currDistmulti;
      totaltilt = TiltAdded+TiltAtShortestDistance;
      SmartDashboard.putNumber("TiltAddPossible", TiltAddPossible);
      SmartDashboard.putNumber("TiltAdded", TiltAdded);
      MaxRotationOffset = kMaxRotationOffset;
      RPMpercentageTolerance = kRpmtolerance*1.5;
      passing = false;
      seeIdColor();
      UpdateOffsetsFromTarget();
      PrepShot();
      return;
    }
    else if(currentPercentOfMaxDistance >= MidDistance & currentPercentOfMaxDistance < 1.0)
    {
      var currDistmulti = currentPercentOfmidToMaxDistance;//Math.abs(Currdistance - (maxDistToShootFrom*MidDistance))/(maxDistToShootFrom- (maxDistToShootFrom*MidDistance));//Math.abs(Currdistance-(maxDistToShootFrom*MidDistance))/(maxDistToShootFrom);
      SmartDashboard.putNumber("currDistmulti", currDistmulti);
      TiltAddPossible = TiltAtMaxDistance-TiltAtMid;
      
      double TiltAdded = TiltAddPossible* currDistmulti;
      totaltilt = TiltAdded+TiltAtMid;
      SmartDashboard.putNumber("TiltAddPossible", TiltAddPossible);
      SmartDashboard.putNumber("TiltAdded", TiltAdded);
      MaxRotationOffset = kMaxRotationOffset; 
      RPMpercentageTolerance = kRpmtolerance*0.9;
      passing = false;
      seeIdColor();
      UpdateOffsetsFromTarget();
      PrepShot();
      return;
    }
    else if(currentPercentOfMaxDistance >= 1.0)
    {
      double blueLineX = 5.0;//5.0;field//1.91;Test#//5.84;
      double redLineX = 11.5;//11.5;field//14.6;Test#//10.9;
      double TiltForPassing = 0.1200;
      double currentX = CurrentPose.getX();
      
      if (currentX > blueLineX & currentX < redLineX)
      {
            SetPassingTarget();
            totaltilt = TiltForPassing;
            TotalRpm = Constants.DeliveryHead.PassingRPM;
            m_candleSubsystem.GreenLights();
            passing = true;
            MaxRotationOffset = kMaxRotationOffset*passingrotationMulti;
            RPMpercentageTolerance = kRpmtolerance*2;
            UpdateOffsetsFromTarget();
            PrepShot();
            return;
      }
      else
      {
           // totaltilt = 0.0;
            //TotalRpm = 0.0;
            totaltilt = TiltForPassing;
            TotalRpm = Constants.DeliveryHead.PassingRPM;
            MaxRotationOffset = kMaxRotationOffset;
            RPMpercentageTolerance = kRpmtolerance;
            passing = false;
            m_candleSubsystem.StrobeRedLights();
            m_TimerToShoot.restart();
            PrepHead();
            return;
      }
    }
    //end of logic is above
    //// 
      SmartDashboard.putNumber("AlignRotShot-RPM", TotalRpm);
      SmartDashboard.putNumber("AlignRotShot-Tilt", totaltilt);
      SmartDashboard.putNumber("currentPercentOfMaxDistance", currentPercentOfMaxDistance);
    //////

    
    //PrepShot();
    PidTuneRot(AlignRotname);
  }



    public boolean passing = false;

  private void PrepShot() {
    PrepHead();
    drive();
  }



  private void PrepHead() {
    DTilt.setSetpointToPosition(totaltilt);
    Dshooter.SetShootSpeed(TotalRpm);
  }



  private void SetTarget() {
    if(!CurrentAlliance.isPresent()){return;}
    if (CurrentAlliance.get() == Alliance.Red) {
      LimelightHelpers.setPriorityTagID(Constants.LimelightName, Constants.AllianceAprilTags.Red.SpeakerCenter);
      TargetPose = Constants.TargetLocations.Red.SpeakerCenterTagLocation;
    }
    else{
      LimelightHelpers.setPriorityTagID(Constants.LimelightName, Constants.AllianceAprilTags.Blue.SpeakerCenter);
      TargetPose = Constants.TargetLocations.Blue.SpeakerCenterTagLocation;
    }
  }
  private void SetPassingTarget() {
    if(!CurrentAlliance.isPresent()){return;}
    if (CurrentAlliance.get() == Alliance.Red) {
      LimelightHelpers.setPriorityTagID(Constants.LimelightName, Constants.AllianceAprilTags.Red.SpeakerCenter);
      TargetPose = Constants.TargetLocations.Red.PassingLocation;
    }
    else{
      LimelightHelpers.setPriorityTagID(Constants.LimelightName, Constants.AllianceAprilTags.Blue.SpeakerCenter);
      TargetPose = Constants.TargetLocations.Blue.PassingLocation;
    }
  }





  private void seeIdColor() {
    if(tid != Constants.AllianceAprilTags.Red.SpeakerCenter & tid !=Constants.AllianceAprilTags.Blue.SpeakerCenter 
    & tidfront != Constants.AllianceAprilTags.Red.SpeakerCenter & tidfront !=Constants.AllianceAprilTags.Blue.SpeakerCenter)
    {
      m_candleSubsystem.YellowLights();
      //return;
    }
    else
    {
       m_candleSubsystem.RainbowRoadLights();
    }
  }


  


    //get the offset of where we need to go.
  private void UpdateOffsetsFromTarget() {
    //SUBTRACT where we need to go, from where we are. this will give us the translations we need to make 
    double Xpose_Offset = CurrentPose.getX() - TargetPose.getX();
    double Ypose_Offset = CurrentPose.getY() - TargetPose.getY();    
    
    SmartDashboard.putNumber("Ypose_Offset", Ypose_Offset);
    SmartDashboard.putNumber("Xpose_Offset", Xpose_Offset);
    
    //do trig

    Rotation2d turnChassistoangleab = new Rotation2d();
    //we want to aim to the right of the target slightly because we shoot to the left. 
    if (CurrentAlliance.get() == Alliance.Red) {
      boolean areweOnWeakSide = CurrentPose.getY() > TargetPose.getY()+MetersOffCenterForCorrection;
      double offset = Constants.TargetLocations.SpeakerAimOffset;
      if(areweOnWeakSide)
      {
        offset+=Units.inchesToMeters(inchesToAddToAimCorrection);
      }
     turnChassistoangleab = Constants.findAngleBetween(CurrentPose.getX(), CurrentPose.getY(), TargetPose.getX(), TargetPose.getY()-offset);
    }
    else
    {
      boolean areweOnWeakSide = CurrentPose.getY() < TargetPose.getY()-MetersOffCenterForCorrection;
      double offset = Constants.TargetLocations.SpeakerAimOffset;
      if(areweOnWeakSide)
      {
        offset+=Units.inchesToMeters(inchesToAddToAimCorrection);
      }
     turnChassistoangleab = Constants.findAngleBetween(CurrentPose.getX(), CurrentPose.getY(), TargetPose.getX(), TargetPose.getY()+offset);
    }
    //
    double m_positionError = MathUtil.inputModulus(turnChassistoangleab.getDegrees(), -180, 180);
    Rotation2d m_positionError2 = Rotation2d.fromDegrees(m_positionError);
    SmartDashboard.putNumber("turnChassistoangleAbsomod2", m_positionError2.getDegrees());

    Rotation2d RZ_Offset2 = CurrentPose.getRotation().minus(m_positionError2);
    PoseOffset = new Pose2d(Xpose_Offset, Ypose_Offset, RZ_Offset2);
    //SmartDashboard.putNumber("RZ_Offset", RZ_Offset2.getDegrees());
    return;
  }



  private void drive() {

    var RZAdjust = AlignRZController.calculate(PoseOffset.getRotation().getDegrees()) + (Math.signum(AlignRZController.calculate(PoseOffset.getRotation().getDegrees()))*min_RZ_command);
      
    //if(passing & Constants.isRotInTarget(PoseOffset.getRotation(),MaxRotationOffset*1.5)) {RZAdjust = 0;}
      if(Constants.isRotInTarget(PoseOffset.getRotation(),MaxRotationOffset)) {RZAdjust = 0;}//!passing & 
      

      var xSpeed = MathUtil.applyDeadband(XAxis.getAsDouble(),0.08);
      var ySpeed = MathUtil.applyDeadband(YAxis.getAsDouble(),0.08);
      if(DriverStation.isTeleopEnabled())
      {
        drivetrainManager.drivetrain.setControl(drivetrainManager.FCdriveAuton.withVelocityX(-ySpeed* drivetrainManager.MaxSpeed) // Drive forward with // negative Y (forward)
        .withVelocityY(-xSpeed * drivetrainManager.MaxSpeed) // Drive left with negative X (left)
        .withRotationalRate(RZAdjust * drivetrainManager.MaxAngularRate) // Drive counterclockwise with negative X (left)
      );
      }
      else{
         drivetrainManager.drivetrain.setControl(drivetrainManager.FCdriveAuton.withRotationalRate(RZAdjust * drivetrainManager.MaxAngularRate)); // Drive counterclockwise with negative X (left)
      }     
      
  }



    // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    LimelightHelpers.setPriorityTagID(Constants.LimelightName, -1); 
    super.end(interrupted);
  }

  public double maxChassisSpeedToFireMps = 0.5;//meters per second (the units our chassis uses)
  private boolean IsXInTarget() {
    return Math.abs(drivetrainManager.drivetrain.getState().speeds.vxMetersPerSecond) < maxChassisSpeedToFireMps;
  }

  private boolean IsYInTarget() {
    return Math.abs(drivetrainManager.drivetrain.getState().speeds.vyMetersPerSecond) < maxChassisSpeedToFireMps;
  }
  
  @Override
  public boolean isFinished() {
        SmartDashboard.putBoolean("IsPassing", passing);
        RotationInRange = Constants.isRotInTarget(PoseOffset.getRotation(),MaxRotationOffset);//RZoffsetFromSetpoint < MaxRotationOffset;//.01
        boolean isXspeedInTarget = IsXInTarget();
        boolean isYspeedInTarget = IsYInTarget();
        boolean isChassisSpeedInTarget = isXspeedInTarget & isYspeedInTarget;
        SmartDashboard.putBoolean("IsYInTarget", isYspeedInTarget);
        SmartDashboard.putBoolean("IsXInTarget", isXspeedInTarget);
        ///RPM SPOOLer
        RPMReadyTofire = Dshooter.getRPMReadyTofire(RPMpercentageTolerance);
        boolean isHeadTilted = DTilt.atSetpoint(Constants.DeliveryHead.TiltsetpointTolerance,Constants.DeliveryHead.TiltSettleTimeAtPosition);
        ///
        if(RotationInRange && RPMReadyTofire && isHeadTilted && isChassisSpeedInTarget) //currentPercentOfMaxDistance < 100.0  &&
        {
          if(timesgood > goodneeded)
          {
            //timesgood = 0;
            //Stop movement if we are there.
            drivetrainManager.StopDriveTrain();
            //
            m_TimerToShoot.stop();
            SmartDashboard.putNumber("Secs2Fire", m_TimerToShoot.get());
            return true;
          }
          else
          {
            timesgood++;
            return false;
          }
          
        }
      return false;
  }

  //   private boolean isRotInTarget(Rotation2d thisRotation2d,double maxRzOffset) {
  //   double RZoffsetFromSetpoint = (Math.abs(PoseOffset.getRotation().getDegrees())+AlignRzSetpoint);
  //   SmartDashboard.putNumber("RZ_Offset", RZoffsetFromSetpoint);
  //   boolean RotationInRange = RZoffsetFromSetpoint < MaxRotationOffset;
  //   SmartDashboard.putBoolean("isRotInTarget", RotationInRange);
  //   return RotationInRange;
  // }

  // private boolean getRPMReadyTofire() {
  //   boolean isUpperWithinRange = Constants.isWithinPercentage(Dshooter.CurrentEncoderVelocity, Dshooter.LastSetRPM, RPMpercentageTolerance);
  //   boolean islowerWithinRange = Constants.isWithinPercentage(Dshooter.CurrentEncoderVelocity_LowS, Dshooter.LastSetRPM, RPMpercentageTolerance);
  //   SmartDashboard.putBoolean("isUpperWithinRange", isUpperWithinRange);
  //   SmartDashboard.putBoolean("islowerWithinRange", islowerWithinRange);
  //   boolean ReadyTofire = false;
  //   if (isUpperWithinRange && islowerWithinRange)
  //   {
  //     if(m_SettleTimer.get() > SettleTimeAtCorrectRPM)
  //     {
  //       ReadyTofire = true;
  //     }
  //   }
  //   else
  //   {
  //     m_SettleTimer.reset();
  //     m_SettleTimer.start();
  //   }
  //   return ReadyTofire;
  // }

  private void PidTuneRot(String PidName) {
    double p = SmartDashboard.getNumber(PidName + " P Gain", kP);
    double i = SmartDashboard.getNumber(PidName + " I Gain", kI);
    double d = SmartDashboard.getNumber(PidName + " D Gain", kD);
      
    if((p != AlignRZController.getP())) { AlignRZController.setP(p); }
    if((i != AlignRZController.getI())) { AlignRZController.setI(i); }
    if((d != AlignRZController.getD())) { AlignRZController.setD(d); }

  }
}
