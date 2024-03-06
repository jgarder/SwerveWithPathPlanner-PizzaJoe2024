package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.DeliveryHolder;
import frc.robot.subsystems.DrivetrainManager;
import frc.robot.subsystems.Limelight3Subsystem;

public class AlignSpeakerTest extends Command {
      /** Creates a new ArmStopCMD. */
  Limelight3Subsystem limelight3Subsystem;
  DrivetrainManager drivetrainManager;
 double kMaxAngularSpeed = 2;
double kMaxSpeed = 5;
    private final XboxController m_controller = new XboxController(0);
  //private final Drivetrain m_swerve = new Drivetrain();

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);
  public AlignSpeakerTest(DrivetrainManager dtm, Limelight3Subsystem ll3){
    limelight3Subsystem = ll3;
    drivetrainManager=dtm;

        //addRequirements(m_DeliveryHolder);
    }

//   @Override
//   public void autonomousPeriodic() {
//     drive(false);
//     m_swerve.updateOdometry();
//   }

  @Override
  public void execute() {
    drive(true);

  }

  // simple proportional turning control with Limelight.
  // "proportional control" is a control algorithm in which the output is proportional to the error.
  // in this case, we are going to return an angular velocity that is proportional to the 
  // "tx" value from the Limelight.
  double limelight_aim_proportional()
  {    
    // kP (constant of proportionality)
    // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
    // if it is too high, the robot will oscillate.
    // if it is too low, the robot will never reach its target
    // if the robot never turns in the correct direction, kP should be inverted.
    double kP = .055;

    // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
    // your limelight 3 feed, tx should return roughly 31 degrees.
    double targetingAngularVelocity = LimelightHelpers.getTX("limelight") * kP;

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
    var xSpeed =
        -m_xspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftY(), 0.02))
            * kMaxSpeed;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    var ySpeed =
        -m_yspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftX(), 0.02))
            * kMaxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    var rot =
        -m_rotLimiter.calculate(MathUtil.applyDeadband(m_controller.getRightX(), 0.02))
            * kMaxAngularSpeed;

    // while the A-button is pressed, overwrite some of the driving values with the output of our limelight methods
      if(xSpeed > .02)
      {

      }
        final var rot_limelight = limelight_aim_proportional();
        rot = rot_limelight;

        final var forward_limelight = limelight_range_proportional();
        xSpeed = forward_limelight;

        //while using Limelight, turn off field-relative driving.
        fieldRelative = false;


    drivetrainManager.drivetrain.setControl(drivetrainManager.RobotCentricdrive.withVelocityX(xSpeed * drivetrainManager.MaxSpeed) // Drive forward with // negative Y (forward)
            .withVelocityY(ySpeed * drivetrainManager.MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(rot * drivetrainManager.MaxAngularRate) // Drive counterclockwise with negative X (left)
        );

    //SmartDashboard.putNumber("R_Curr", RZ_buffer);
    //SmartDashboard.putNumber("R_PID", RZposeAxis);
    
   
    //SmartDashboard.putNumber("Y_Curr", YP_buffer);
    //SmartDashboard.putNumber("Y_PID", forward_limelight);

    //SmartDashboard.putNumber("X_Curr", XP_buffer);
    //SmartDashboard.putNumber("X_PID", xSpeed);
    //m_swerve.drive(xSpeed, ySpeed, rot, fieldRelative, getPeriod());
  }
}
