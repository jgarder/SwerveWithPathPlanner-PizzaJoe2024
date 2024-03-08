package frc.robot.commands;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
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
  
  final double TiltAtShortestDistance = Constants.DeliveryHead.Tilt_Position_Speaker_Closest;
  final double TiltAtMaxDistance = Constants.DeliveryHead.Tilt_Position_Speaker_Podium;//Tilt at max distance (podium)

  final double RPMatShortestDistance = Constants.DeliveryHead.ShooterRpmSpeakerClose;
  final double RPMAtMaxDistance = Constants.DeliveryHead.ShooterRpmSpeakerPodium;//RPM at max distance (podium)
  

  double RpmAddPossible = RPMAtMaxDistance-RPMatShortestDistance;
  double TiltAddPossible = TiltAtMaxDistance-TiltAtShortestDistance;
  private final PIDController AlignRZController = new PIDController(Constants.ChassisPid.k_RZ_P,Constants.ChassisPid.k_RZ_I,Constants.ChassisPid.k_RZ_D);
  Optional<Alliance> CurrentAlliance;
  public String AlignRotname = "AlignRotShot";
  double RPMpercentageTolerance = 4;
  int timesgood = 0;
  int goodneeded = 5;
  boolean RotationInRange = false;
  double MaxRotationOffset = .025;
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
  
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //get our alliance red or blue.
   CurrentAlliance = DriverStation.getAlliance();
   AlignRZController.setSetpoint(-13); //-13 means we always look to the right of where we want to shoot. 
  }
  @Override
  public void execute() {
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
    if (currentPercentOfMaxDistance > 1.0) //if we are at greater than 100% shooting
    {
      m_candleSubsystem.RedLights();
      return;
    }
    ///////////////////////
    if(!CurrentAlliance.isPresent()){return;}
    if (CurrentAlliance.get() == Alliance.Red) {
      LimelightHelpers.setPriorityTagID("limelight", 4);
    }
    else{
      LimelightHelpers.setPriorityTagID("limelight", 7);
    }
    double tid = LimelightHelpers.getFiducialID("limelight");
    if(tid != 4 & tid !=7)
    {
      m_candleSubsystem.StrobeRedLights();
      return;
    }
    else
    {
       m_candleSubsystem.RainbowRoadLights();
    }

    DTilt.setSetpointToPosition(totaltilt);
    Dshooter.SetShootSpeed(TotalRpm);
    drive();

  }
  
  double limelight_aim_proportional()
  {    
    // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
    // your limelight 3 feed, tx should return roughly 31 degrees.
    double targetingAngular = LimelightHelpers.getTX("limelight");
    double RZAdjust = AlignRZController.calculate(targetingAngular);


    return RZAdjust;//targetingAngularVelocity;
  }

  private void drive() {

      var xSpeed = MathUtil.applyDeadband(XAxis.getAsDouble(),0.02);
      var ySpeed = MathUtil.applyDeadband(YAxis.getAsDouble(),0.02);     
      drivetrainManager.drivetrain.setControl(drivetrainManager.FCdriveAuton.withVelocityX(-ySpeed* drivetrainManager.MaxSpeed) // Drive forward with // negative Y (forward)
        .withVelocityY(-xSpeed * drivetrainManager.MaxSpeed) // Drive left with negative X (left)
        .withRotationalRate(rotationOffset * drivetrainManager.MaxAngularRate) // Drive counterclockwise with negative X (left)
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

        RotationInRange = Math.abs(rotationOffset) < MaxRotationOffset;//.01
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
