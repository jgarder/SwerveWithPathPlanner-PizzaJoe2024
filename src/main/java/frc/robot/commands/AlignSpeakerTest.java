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
      
  DrivetrainManager drivetrainManager;
  CANdleSystem m_candleSubsystem;
  DeliveryTilt DTilt;
  DeliveryShooter Dshooter;
  double kMaxAngularSpeed = 2;
  double kMaxSpeed = 5;


  DoubleSupplier XAxis;
  DoubleSupplier YAxis;

  double rotationOffset = 0;
  double totaltilt=0;
  double TotalRpm=0;
  double currentPercentOfMaxDistance = 0;

  final double minYdistanceToShootFrom = 1.20;
  final double maxYtoShootFrom = 23.2;
  
   double TiltAtShortestDistance = 0;//Constants.DeliveryHead.Tilt_Position_Speaker_Closest;
   double TiltAtMaxDistance = 0;//Constants.DeliveryHead.Tilt_Position_Speaker_Furthest;//Tilt at max distance (podium)

  final double RPMatShortestDistance = Constants.DeliveryHead.ShooterRpmSpeakerClose;
  final double RPMAtMaxDistance = Constants.DeliveryHead.ShooterRpmSpeakerPodium;//RPM at max distance (podium)
  

  double RpmAddPossible = RPMAtMaxDistance-RPMatShortestDistance;
  double TiltAddPossible = TiltAtMaxDistance-TiltAtShortestDistance;
  private final PIDController AlignRZController = new PIDController(.003,Constants.ChassisPid.k_RZ_I,Constants.ChassisPid.k_RZ_D);
  Optional<Alliance> CurrentAlliance;
  public String AlignRotname = "AlignRotShot";
  double RPMpercentageTolerance = 4;
  int timesgood = 0;
  int goodneeded = 5;
  boolean RotationInRange = false;
  double MaxRotationOffset = 1.43;
  private final Timer m_SettleTimer = new Timer();
  double SettleTimeAtCorrectRPM = .15;


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
  
  double AlignRzSetpoint = -1;

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //get our alliance red or blue.
   CurrentAlliance = DriverStation.getAlliance();
   AlignRZController.setSetpoint(AlignRzSetpoint); //-13 means we always look to the right of where we want to shoot. 

    TiltAtShortestDistance = Constants.DeliveryHead.Tilt_Position_Speaker_Closest;
    TiltAtMaxDistance = Constants.DeliveryHead.Tilt_Position_Speaker_Furthest;//Tilt at max distance (podium)
    TiltAddPossible = TiltAtMaxDistance-TiltAtShortestDistance;
    //System.out.println("Closest num" + Constants.DeliveryHead.Tilt_Position_Speaker_Closest);
  }
  @Override
  public void execute() {
    //update current pose
    CurrentPose = drivetrainManager.drivetrain.getState().Pose;

    //distance check
    ///
    var Currdistance = LimelightHelpers.getTY("limelight");
    currentPercentOfMaxDistance = Math.abs(Currdistance)/maxYtoShootFrom;
    /////
    //get aim and offsets
     final var rot_limelight = limelight_aim_proportional();
      rotationOffset = rot_limelight;
      SmartDashboard.putNumber("RotationOffsetAiming", rotationOffset);

      
      double RpmAdded = RpmAddPossible* currentPercentOfMaxDistance;
      //SmartDashboard.putNumber("AlignRotShot-RpmAddPossible", RpmAddPossible);
      //SmartDashboard.putNumber("AlignRotShot-RPMadded", RpmAdded);
      //SmartDashboard.putNumber("AlignRotShot-RPMatShortestDistance", RPMatShortestDistance);
      double TiltAdded = TiltAddPossible* currentPercentOfMaxDistance;
      totaltilt = TiltAdded+TiltAtShortestDistance;
      TotalRpm = RpmAdded+RPMatShortestDistance;
      SmartDashboard.putNumber("AlignRotShot-RPM", TotalRpm);
      SmartDashboard.putNumber("AlignRotShot-Tilt", totaltilt);
      SmartDashboard.putNumber("currentPercentOfMaxDistance", currentPercentOfMaxDistance);
    //////
    ///////////////////////
    if(!CurrentAlliance.isPresent()){return;}
    if (CurrentAlliance.get() == Alliance.Red) {
      LimelightHelpers.setPriorityTagID("limelight", 4);
      TargetPose = Constants.TargetLocations.Red.SpeakerCenterTagLocation;
    }
    else{
      LimelightHelpers.setPriorityTagID("limelight", 7);
      TargetPose = Constants.TargetLocations.Blue.SpeakerCenterTagLocation;
    }
    //
    
    UpdateOffsetsFromTarget();

    

    //
    //
    //
    if (currentPercentOfMaxDistance > 1.0) //if we are at greater than 100% shooting
    {
      m_candleSubsystem.StrobeRedLights();
      //return;
    }
    //
    //
    double tid = LimelightHelpers.getFiducialID("limelight");
    if(tid != 4 & tid !=7)
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

  }
  
  public Rotation2d findAngleBetween ( double x1 ,double y1 ,double x2 ,double y2 )
{
     var calc_angle = Math.atan2 ( y2 - y1 , x2 - x1 ) ;
     // notice y is the first parameter not x
     // y is the rise and x is the run
     // we could do (y2-y1, x2-x1) or (y1-y2, x1-x2)
     if ( calc_angle < 0 ) // we don't want negative angles
    {
         calc_angle += Math.PI * 2;
         // make negative angles positive by adding 360 degrees
    }
    // convert angle from radians to degrees then log
    var thisangle = calc_angle * ( 180 / Math.PI );
    SmartDashboard.putNumber("angle raw", thisangle);
     return new Rotation2d(calc_angle);
} 

    //get the offset of where we need to go.
  private void UpdateOffsetsFromTarget() {
    //SUBTRACT where we need to go, from where we are. this will give us the translations we need to make 
    double Xpose_Offset = CurrentPose.getX() - TargetPose.getX();
    double Ypose_Offset = CurrentPose.getY() - TargetPose.getY();            
    
    //do trig
    Rotation2d turnChassistoangleab = findAngleBetween(CurrentPose.getX(), CurrentPose.getY(), TargetPose.getX(), TargetPose.getY());
    double m_positionError = MathUtil.inputModulus(turnChassistoangleab.getDegrees(), -180, 180);
    Rotation2d m_positionError2 = Rotation2d.fromDegrees(m_positionError);
    SmartDashboard.putNumber("turnChassistoangleAbsomod2", m_positionError2.getDegrees());

    Rotation2d RZ_Offset2 = CurrentPose.getRotation().minus(m_positionError2);
    PoseOffset = new Pose2d(Xpose_Offset, Ypose_Offset, RZ_Offset2);
    //SmartDashboard.putNumber("RZ_Offset", RZ_Offset2.getDegrees());
    return;
  }

  double limelight_aim_proportional()
  {    
    // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
    // your limelight 3 feed, tx should return roughly 31 degrees.
    double targetingAngular = LimelightHelpers.getTX("limelight");
    double RZAdjust = AlignRZController.calculate(targetingAngular);


    return RZAdjust;//targetingAngularVelocity;
  }
  double min_RZ_command = Constants.ChassisPid.min_RZ_command;
  private void drive() {

    var RZAdjust = AlignRZController.calculate(PoseOffset.getRotation().getDegrees()) + (Math.signum(AlignRZController.calculate(PoseOffset.getRotation().getDegrees()))*min_RZ_command);

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
        double RZoffsetFromSetpoint = (Math.abs(PoseOffset.getRotation().getDegrees())+AlignRzSetpoint);
        SmartDashboard.putNumber("RZ_Offset", RZoffsetFromSetpoint);
        RotationInRange = RZoffsetFromSetpoint < MaxRotationOffset;//.01
        SmartDashboard.putBoolean("isRotInTarget", RotationInRange);
        ///RPM SPOOLer
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
}
