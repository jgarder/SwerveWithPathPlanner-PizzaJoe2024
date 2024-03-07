package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.DeliveryShooter;
import frc.robot.subsystems.DeliveryTilt;
import frc.robot.subsystems.DrivetrainManager;
import frc.robot.subsystems.Limelight3Subsystem;

public class AlignSpeakerTest extends Command {
      /** Creates a new ArmStopCMD. */
  Limelight3Subsystem limelight3Subsystem;
  DrivetrainManager drivetrainManager;
  DeliveryTilt DTilt;
  DeliveryShooter Dshooter;
 double kMaxAngularSpeed = 2;
double kMaxSpeed = 5;
  //private final Drivetrain m_swerve = new Drivetrain();

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  DoubleSupplier XAxis;
  DoubleSupplier YAxis;
  public AlignSpeakerTest(DrivetrainManager dtm, DeliveryTilt dTilt,DeliveryShooter dshooter,Limelight3Subsystem ll3,DoubleSupplier Xaxis, DoubleSupplier Yaxis){
    DTilt = dTilt;
    Dshooter = dshooter;
    XAxis = Xaxis;
    YAxis =  Yaxis;
    limelight3Subsystem = ll3;
    drivetrainManager=dtm;

        //addRequirements(m_DeliveryHolder);
    }

//   @Override
//   public void autonomousPeriodic() {
//     drive(false);
//     m_swerve.updateOdometry();
//   }
  public String AlignRotname = "AlignRotShot";
  @Override
  public void execute() {
    drive(true);

    PidTuneRot(AlignRotname);
    SmartDashboard.putNumber(AlignRotname + " P Gain", kP);
  }
  // kP (constant of proportionality)
  // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
  // if it is too high, the robot will oscillate.
  // if it is too low, the robot will never reach its target
  // if the robot never turns in the correct direction, kP should be inverted.
  double kP = .045;
  // simple proportional turning control with Limelight.
  // "proportional control" is a control algorithm in which the output is proportional to the error.
  // in this case, we are going to return an angular velocity that is proportional to the 
  // "tx" value from the Limelight.
  double limelight_aim_proportional()
  {    
    // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
    // your limelight 3 feed, tx should return roughly 31 degrees.
    double targetingAngular = LimelightHelpers.getTX("limelight");
    if(targetingAngular > 0)
    {
      targetingAngular -= 10;
    }
    if(targetingAngular < 0)
    {
      targetingAngular += 10;
    }
    
    double targetingAngularVelocity = targetingAngular * kP;

    // convert to radians per second for our drive method
    targetingAngularVelocity *= kMaxAngularSpeed;

    //invert since tx is positive when the target is to the right of the crosshair
    targetingAngularVelocity *= -1.0;

    return targetingAngularVelocity;
  }

    // simple proportional ranging control with Limelight's "ty" value
    // this works best if your Limelight's mount height and target mount height are different.
    // if your limelight and target are mounted at the same or similar heights, use "ta" (area) for target ranging rather than "ty"
    double yoffset = 0;
    double limelight_range_proportional()
    {    
        double kP = .1;
        double targetingForwardSpeed = LimelightHelpers.getTY("limelight") * kP;
        targetingForwardSpeed *= kMaxSpeed;
        targetingForwardSpeed *= -1.0;
        return targetingForwardSpeed;
    }


    private void drive(boolean fieldRelative) {

    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    // var xSpeed =
    //     -m_xspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftY(), 0.02))
    //         * kMaxSpeed;

    // // Get the y speed or sideways/strafe speed. We are inverting this because
    // // we want a positive value when we pull to the left. Xbox controllers
    // // return positive values when you pull to the right by default.
    // var ySpeed =
    //     -m_yspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftX(), 0.02))
    //         * kMaxSpeed;

    // // Get the rate of angular rotation. We are inverting this because we want a
    // // positive value when we pull to the left (remember, CCW is positive in
    // // mathematics). Xbox controllers return positive values when you pull to
    // // the right by default.
    // var rot =
    //     -m_rotLimiter.calculate(MathUtil.applyDeadband(m_controller.getRightX(), 0.02))
    //         * kMaxAngularSpeed;

    // while the A-button is pressed, overwrite some of the driving values with the output of our limelight methods

        final var rot_limelight = limelight_aim_proportional();
        rotationOffset = rot_limelight;
        SmartDashboard.putNumber("RotationOffsetAiming", rotationOffset);

        //final var forward_limelight = limelight_range_proportional();
        var Currdistance = LimelightHelpers.getTY("limelight");

        currentPercentOfMaxDistance = Math.abs(Currdistance)/maxYtoShootFrom;
        double RpmAdded = RpmAddPossible* currentPercentOfMaxDistance;
        double TiltAdded = TiltAddPossible* currentPercentOfMaxDistance;
        totaltilt = TiltAdded+TiltAtShortestDistance;
        TotalRpm = RpmAdded+RPMatShortestDistance;
        SmartDashboard.putNumber("AlignRotShot-RPM", TotalRpm);
        SmartDashboard.putNumber("AlignRotShot-Tilt", totaltilt);
        SmartDashboard.putNumber("currentPercentOfMaxDistance", currentPercentOfMaxDistance);
          var xSpeed = MathUtil.applyDeadband(XAxis.getAsDouble(),0.02);

          var ySpeed = MathUtil.applyDeadband(YAxis.getAsDouble(),0.02);
        
        DTilt.setSetpointToPosition(totaltilt);
        Dshooter.SetShootSpeed(TotalRpm);
      
    drivetrainManager.drivetrain.setControl(drivetrainManager.FCdriveAuton.withVelocityX(-ySpeed* drivetrainManager.MaxSpeed) // Drive forward with // negative Y (forward)
            .withVelocityY(-xSpeed * drivetrainManager.MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(rotationOffset * drivetrainManager.MaxAngularRate) // Drive counterclockwise with negative X (left)
        );

  }
  double rotationOffset = 0;
  double totaltilt=0;
  double TotalRpm=0;
  double currentPercentOfMaxDistance = 0;
  double minYdistanceToShootFrom = 1.20;
  double maxYtoShootFrom = 22;
  
  double TiltAtShortestDistance = Constants.DeliveryHead.Tilt_Position_Speaker_Closest;
  double TiltAtMaxDistance = Constants.DeliveryHead.Tilt_Position_Speaker_Podium;//Tilt at max distance (podium)

  double RPMatShortestDistance = Constants.DeliveryHead.ShooterRpmSpeakerClose;
  double RPMAtMaxDistance = Constants.DeliveryHead.ShooterRpmSpeakerPodium;//RPM at max distance (podium)
  

  double RpmAddPossible = RPMAtMaxDistance-RPMatShortestDistance;
  double TiltAddPossible = TiltAtMaxDistance-TiltAtShortestDistance;

  
  //double currentPercentOfMaxDistance = 0;//MAX distance is the podium for our robot
  //how many times must robot be in location for this to command to finish.
  double percentageTolerance = 5;
  int timesgood = 0;
  int goodneeded = 5;
  boolean RotationInRange = false;
  private final Timer m_SettleTimer = new Timer();
  double SettleTimeAtCorrectRPM = .25;
  @Override
  public boolean isFinished() {

        RotationInRange = Math.abs(rotationOffset) < .025;//.01
        SmartDashboard.putBoolean("isRotInTarget", RotationInRange);
        ///RPM SPOOLer
        boolean isUpperWithinRange = Constants.isWithinPercentage(Dshooter.CurrentEncoderVelocity, Dshooter.LastSetRPM, percentageTolerance);
        boolean islowerWithinRange = Constants.isWithinPercentage(Dshooter.CurrentEncoderVelocity_LowS, Dshooter.LastSetRPM, percentageTolerance);
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
        if(currentPercentOfMaxDistance < 100  && RotationInRange && ReadyTofire)
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
  private void PidTuneRot(String PidName) {
    double p = SmartDashboard.getNumber(PidName + " P Gain", Constants.ChassisPid.k_RZ_P);
    if((p != kP)) { kP = p; }

  }
}
