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
      

  double kP = 0.025;
  double kI = .00010;
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

    double MaxRotationOffset = 2.0;//2.0;//1.43;
  
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
  double RPMpercentageTolerance = 4;
  int timesgood = 0;
  int goodneeded = 5;

  boolean RotationInRange = false;

  private final Timer m_SettleTimer = new Timer();
  double SettleTimeAtCorrectRPM = .15;
  double min_RZ_command = Constants.ChassisPid.min_RZ_command;


  public AlignSpeakerTest(DrivetrainManager dtm, DeliveryTilt dTilt,DeliveryShooter dshooter,CANdleSystem candleSubsystem,DoubleSupplier Xaxis, DoubleSupplier Yaxis){
    m_candleSubsystem = candleSubsystem;
    DTilt = dTilt;
    Dshooter = dshooter;
    XAxis = Xaxis;
    YAxis =  Yaxis;
    drivetrainManager=dtm;
  
        //addRequirements(m_DeliveryHolder);
    }
    
  Pose2d CurrentPose;//this is our latest position according to our chassis odometry

  Pose2d TargetPose;//this is where we wnt to go in field space coords X,y,Rotation

  Pose2d PoseOffset;//This is how far we are from where we want to be. this is CurrentPose minus TargetPose.
  
  double AlignRzSetpoint = 0;//-1;//-13 means we always look to the right of where we want to shoot.

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //get our alliance red or blue.
   CurrentAlliance = DriverStation.getAlliance();
   AlignRZController.setSetpoint(AlignRzSetpoint);  

    TiltAtShortestDistance = Constants.DeliveryHead.Tilt_Position_Speaker_Closest;
    TiltAtMid = Constants.DeliveryHead.Tilt_Position_Speaker_Mid;
    TiltAtMaxDistance = Constants.DeliveryHead.Tilt_Position_Speaker_Furthest;//Tilt at max distance (podium)
    
    //System.out.println("Closest num" + Constants.DeliveryHead.Tilt_Position_Speaker_Closest);
    /////////////////
    PidTuneRot(AlignRotname);
    SmartDashboard.putNumber(AlignRotname + " P Gain", AlignRZController.getP());
    SmartDashboard.putNumber(AlignRotname + " I Gain", AlignRZController.getI());
    SmartDashboard.putNumber(AlignRotname + " D Gain", AlignRZController.getD());
    ///////////////
  }
  @Override
  public void execute() {
    
    //update current pose
    CurrentPose = drivetrainManager.drivetrain.getState().Pose;
    ///
    //////check alliance and set target
    if(!CurrentAlliance.isPresent()){return;}
    if (CurrentAlliance.get() == Alliance.Red) {
      LimelightHelpers.setPriorityTagID("limelight", 4);
      TargetPose = Constants.TargetLocations.Red.SpeakerCenterTagLocation;
    }
    else{
      LimelightHelpers.setPriorityTagID("limelight", 7);
      TargetPose = Constants.TargetLocations.Blue.SpeakerCenterTagLocation;
    }
    // get offsets of current pose from target pose
    double MidDistance = .5;
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
    var MaxDistMultiplier = MathUtil.clamp(currentPercentOfMaxDistance, 0.000, 1.000);
    SmartDashboard.putNumber("MaxDistMultiplier", MaxDistMultiplier);
    ///
    double RpmAdded = RpmAddPossible* MaxDistMultiplier;
    TotalRpm = RpmAdded+RPMatShortestDistance;
    /////
  
    if(currentPercentOfMaxDistance < MidDistance)
    {
      var currDistmulti = currentPercentOfMidDistance;//Math.abs(Currdistance - minDistanceToShootFrom)/(maxDistToShootFrom- minDistanceToShootFrom);// Math.abs(Currdistance)/(maxDistToShootFrom*MidDistance);
      SmartDashboard.putNumber("currDistmulti", currDistmulti);
      TiltAddPossible = TiltAtMid-TiltAtShortestDistance;
      double TiltAdded = TiltAddPossible* currDistmulti;
      totaltilt = TiltAdded+TiltAtShortestDistance;
      SmartDashboard.putNumber("TiltAddPossible", TiltAddPossible);
      SmartDashboard.putNumber("TiltAdded", TiltAdded);
    }
    else
    {
      var currDistmulti = currentPercentOfmidToMaxDistance;//Math.abs(Currdistance - (maxDistToShootFrom*MidDistance))/(maxDistToShootFrom- (maxDistToShootFrom*MidDistance));//Math.abs(Currdistance-(maxDistToShootFrom*MidDistance))/(maxDistToShootFrom);
      SmartDashboard.putNumber("currDistmulti", currDistmulti);
      TiltAddPossible = TiltAtMaxDistance-TiltAtMid;
      
      double TiltAdded = TiltAddPossible* currDistmulti;
      totaltilt = TiltAdded+TiltAtMid;
      SmartDashboard.putNumber("TiltAddPossible", TiltAddPossible);
      SmartDashboard.putNumber("TiltAdded", TiltAdded);
    }
    //// 
    //get aim and offsets
     //final var rot_limelight = limelight_aim_proportional();
     // rotationOffset = rot_limelight;
     // SmartDashboard.putNumber("RotationOffsetAiming", rotationOffset);
      //SmartDashboard.putNumber("AlignRotShot-RpmAddPossible", RpmAddPossible);
      //SmartDashboard.putNumber("AlignRotShot-RPMadded", RpmAdded);
      //SmartDashboard.putNumber("AlignRotShot-RPMatShortestDistance", RPMatShortestDistance);
      
      
      
      SmartDashboard.putNumber("AlignRotShot-RPM", TotalRpm);
      SmartDashboard.putNumber("AlignRotShot-Tilt", totaltilt);
      SmartDashboard.putNumber("currentPercentOfMaxDistance", currentPercentOfMaxDistance);
    //////

    //
    double tid = LimelightHelpers.getFiducialID("limelight");
    if (currentPercentOfMaxDistance > 1.0) //if we are at greater than 100% shooting
    {
      m_candleSubsystem.StrobeRedLights();
      //return;
    }
    else if(tid != 4 & tid !=7)
    {
      m_candleSubsystem.YellowLights();
      //return;
    }
    else
    {
       m_candleSubsystem.RainbowRoadLights();
    }
    //
    DTilt.setSetpointToPosition(totaltilt);
    Dshooter.SetShootSpeed(TotalRpm);
    //
    // var xSpeed = MathUtil.applyDeadband(XAxis.getAsDouble(),0.02);
    // var ySpeed = MathUtil.applyDeadband(YAxis.getAsDouble(),0.02);  
       
    //   drivetrainManager.drivetrain.setControl(drivetrainManager.fieldpoint.withVelocityX(-ySpeed* drivetrainManager.MaxSpeed) // Drive forward with // negative Y (forward)
    //     .withVelocityY(-xSpeed * drivetrainManager.MaxSpeed) // Drive left with negative X (left)
    //     .withTargetDirection(m_positionError2) // Drive counterclockwise with negative X (left)
    // );
    //
    drive();
    PidTuneRot(AlignRotname);
  }
  


    //get the offset of where we need to go.
  private void UpdateOffsetsFromTarget() {
    //SUBTRACT where we need to go, from where we are. this will give us the translations we need to make 
    double Xpose_Offset = CurrentPose.getX() - TargetPose.getX();
    double Ypose_Offset = CurrentPose.getY() - TargetPose.getY();    
    
    SmartDashboard.putNumber("Ypose_Offset", Ypose_Offset);
    SmartDashboard.putNumber("Xpose_Offset", Xpose_Offset);
    
    //do trig
    Rotation2d turnChassistoangleab = Constants.findAngleBetween(CurrentPose.getX(), CurrentPose.getY(), TargetPose.getX(), TargetPose.getY());
    double m_positionError = MathUtil.inputModulus(turnChassistoangleab.getDegrees(), -180, 180);
    Rotation2d m_positionError2 = Rotation2d.fromDegrees(m_positionError);
    SmartDashboard.putNumber("turnChassistoangleAbsomod2", m_positionError2.getDegrees());

    Rotation2d RZ_Offset2 = CurrentPose.getRotation().minus(m_positionError2);
    PoseOffset = new Pose2d(Xpose_Offset, Ypose_Offset, RZ_Offset2);
    //SmartDashboard.putNumber("RZ_Offset", RZ_Offset2.getDegrees());
    return;
  }

  // double limelight_aim_proportional()
  // {    
  //   // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
  //   // your limelight 3 feed, tx should return roughly 31 degrees.
  //   double targetingAngular = LimelightHelpers.getTX("limelight");
  //   double RZAdjust = AlignRZController.calculate(targetingAngular);


  //   return RZAdjust;//targetingAngularVelocity;
  // }

  private void drive() {

    var RZAdjust = AlignRZController.calculate(PoseOffset.getRotation().getDegrees()) + (Math.signum(AlignRZController.calculate(PoseOffset.getRotation().getDegrees()))*min_RZ_command);

      if(isRotInTarget()) {RZAdjust = 0;}

      var xSpeed = MathUtil.applyDeadband(XAxis.getAsDouble(),0.02);
      var ySpeed = MathUtil.applyDeadband(YAxis.getAsDouble(),0.02);     
      drivetrainManager.drivetrain.setControl(drivetrainManager.FCdriveAuton.withVelocityX(-ySpeed* drivetrainManager.MaxSpeed) // Drive forward with // negative Y (forward)
        .withVelocityY(-xSpeed * drivetrainManager.MaxSpeed) // Drive left with negative X (left)
        .withRotationalRate(RZAdjust * drivetrainManager.MaxAngularRate) // Drive counterclockwise with negative X (left)
    );
  }



    // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    LimelightHelpers.setPriorityTagID("limelight", -1); 
    super.end(interrupted);
  }

  
  @Override
  public boolean isFinished() {
        
        RotationInRange = isRotInTarget();//RZoffsetFromSetpoint < MaxRotationOffset;//.01

        ///RPM SPOOLer
        boolean ReadyTofire = getRPMReadyTofire();
        ///
        if(currentPercentOfMaxDistance < 100  && RotationInRange && ReadyTofire && DTilt.atSetpoint())
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
      return false;
  }

    private boolean isRotInTarget() {
    double RZoffsetFromSetpoint = (Math.abs(PoseOffset.getRotation().getDegrees())+AlignRzSetpoint);
    SmartDashboard.putNumber("RZ_Offset", RZoffsetFromSetpoint);
    boolean RotationInRange = RZoffsetFromSetpoint < MaxRotationOffset;
    SmartDashboard.putBoolean("isRotInTarget", RotationInRange);
    return RotationInRange;
  }

  private boolean getRPMReadyTofire() {
    boolean isUpperWithinRange = Constants.isWithinPercentage(Dshooter.CurrentEncoderVelocity, Dshooter.LastSetRPM, RPMpercentageTolerance);
    boolean islowerWithinRange = Constants.isWithinPercentage(Dshooter.CurrentEncoderVelocity_LowS, Dshooter.LastSetRPM, RPMpercentageTolerance);
    SmartDashboard.putBoolean("isUpperWithinRange", isUpperWithinRange);
    SmartDashboard.putBoolean("islowerWithinRange", islowerWithinRange);
    boolean ReadyTofire = false;
    if (isUpperWithinRange && islowerWithinRange)
    {
      if(m_SettleTimer.get() > SettleTimeAtCorrectRPM)
      {
        ReadyTofire = true;
      }
    }
    else
    {
      m_SettleTimer.reset();
      m_SettleTimer.start();
    }
    return ReadyTofire;
  }

  private void PidTuneRot(String PidName) {
    double p = SmartDashboard.getNumber(PidName + " P Gain", kP);
    double i = SmartDashboard.getNumber(PidName + " I Gain", kI);
    double d = SmartDashboard.getNumber(PidName + " D Gain", kD);
      
    if((p != AlignRZController.getP())) { AlignRZController.setP(p); }
    if((i != AlignRZController.getI())) { AlignRZController.setI(i); }
    if((d != AlignRZController.getD())) { AlignRZController.setD(d); }

  }
}
