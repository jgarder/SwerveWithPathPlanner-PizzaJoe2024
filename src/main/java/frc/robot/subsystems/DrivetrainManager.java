package frc.robot.subsystems;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Telemetry;
import frc.robot.RobotContainer.PizzaManager;
import frc.robot.generated.TunerConstants;

public class DrivetrainManager extends SubsystemBase{
    
  public final double kMaxSpeed = 4.3; //closer the max speed is to the max speed in real life the more resolution the joystick will have for driver. 
  public double MaxSpeed = kMaxSpeed;//9;//6; // 6 meters per second desired top speed
  public double MaxAngularRate = 2.5 * Math.PI; //1.8 ==  3/4 of a rotation per second max angular velocity

  public double MaxSpeedPid = 9;//6; // 6 meters per second desired top speed
  public double MaxAngularRatePid = 2.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  
    private CommandXboxController joystick;
    //private CommandSwerveDrivetrain drivetrain;
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
    private final Telemetry logger = new Telemetry(MaxSpeed);


      /* Path follower */
    public Command runAuto;
    

    public DrivetrainManager(CommandXboxController Joystick) {
        joystick = Joystick;


        //drivetrain = Drivetrain;
    }
    @Override
    public void periodic() {
      double VelocX = speedLimiterY.calculate(-joystick.getLeftY()) * (MaxSpeed*PizzaManager.speedMulti);
      double VelocY = speedLimiterX.calculate(-joystick.getLeftX()) * (MaxSpeed*PizzaManager.speedMulti);
      SmartDashboard.putNumber("Joystick-VelocX", VelocX);
      SmartDashboard.putNumber("Joystick-VelocY", VelocY);

     // SmartDashboard.putNumber("AccumGyroZ", drivetrain.getPigeon2().getAccumGyroZ().getValueAsDouble() - accumGyroz);
      //SmartDashboard.putNumber("GyroYaw", drivetrain.getPigeon2().getYaw().getValueAsDouble());
    }
    public double accumGyroz = 0;
    public static final double slewrateXY = 9.0;//10.0;//9;//7;//20;//3;
    public static final double slewrateRotation = 20;//2.7;
    SlewRateLimiter speedLimiterX = new SlewRateLimiter(slewrateXY);
    SlewRateLimiter speedLimiterY = new SlewRateLimiter(slewrateXY);
    SlewRateLimiter speedLimiterRotation = new SlewRateLimiter(slewrateRotation);
    public void configureBindings()
    {

      //////////////////////////////////////
        drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> FCdrive.withVelocityX(speedLimiterY.calculate(-joystick.getLeftY()) * (MaxSpeed*PizzaManager.speedMulti)) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(speedLimiterX.calculate(-joystick.getLeftX()) * (MaxSpeed*PizzaManager.speedMulti)) // Drive left with negative X (left)
            .withRotationalRate(speedLimiterRotation.calculate(-joystick.getRightX()) * (MaxAngularRate*PizzaManager.RotationMulti)) // Drive counterclockwise with negative X (left)
        ).ignoringDisable(true));
      joystick.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));
        //////////////////////////////////

        //  //////////////////THIS IS ROBOT CENTRIC TESTING MODE////////////////////
        // drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        // drivetrain.applyRequest(() -> RobotCentricdrive.withVelocityX(speedLimiterY.calculate(-joystick.getLeftY()) * (MaxSpeed*PizzaManager.speedMulti)) // Drive forward with
        //                                                                                    // negative Y (forward)
        //     .withVelocityY(speedLimiterX.calculate(-joystick.getLeftX()) * (0*PizzaManager.speedMulti)) // Drive left with negative X (left)
        //     .withRotationalRate(speedLimiterRotation.calculate(-joystick.getRightX()) * (0*PizzaManager.RotationMulti)) // Drive counterclockwise with negative X (left)
        // ).ignoringDisable(true));
        // joystick.start().onTrue(drivetrain.runOnce(() -> PizzaManager.LimelightTelemetryUpdateRequested = !PizzaManager.LimelightTelemetryUpdateRequested));
        // //////////////////////////////////

        //stuff below should be tested when drivetrain is complete    
        //joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
       // joystick.b().whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));
       // joystick.b().whileTrue(new InstantCommand(()->{drivetrain.getPigeon2().setYaw(0);accumGyroz = drivetrain.getPigeon2().getAccumGyroZ().getValueAsDouble();}));
        //fieldpoint.HeadingController = new PhoenixPIDController(4, 0, 0);
        //joystick.pov(0).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(0)));
        //joystick.pov(180).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)));
        
        // reset the field-centric heading on start press
        //joystick.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

        // if (Utils.isSimulation()) {
        //   drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
        // }
        drivetrain.registerTelemetry(logger::telemeterize);


    }

    public void MoveRobotToTargetInFieldCoordinates(double YposeAxis, double XposeAxis, double RZposeAxis) {
      drivetrain.setControl(FCdriveAuton.withVelocityX(XposeAxis * MaxSpeedPid) // Drive forward with // negative Y (forward)
          .withVelocityY(YposeAxis * MaxSpeedPid) // Drive left with negative X (left)
          .withRotationalRate(RZposeAxis * MaxAngularRatePid) // Drive counterclockwise with negative X (left)
      );
    }
    public void StopDriveTrain() {
      drivetrain.setControl(RobotCentricdrive.withVelocityX(0 * MaxSpeed) // Drive forward with // negative Y (forward)
      .withVelocityY(0 * MaxSpeed) // Drive left with negative X (left)
      .withRotationalRate(0 * MaxAngularRate) // Drive counterclockwise with negative X (left)
      );
    }

    // public void move(double x,double y,double r)
    // {
    //   drivetrain.applyRequest(() -> FCdrive.withVelocityX(-.1 * MaxSpeed) // Drive forward with                                                                                        // negative Y (forward)
    //         .withVelocityY(-.1 * MaxSpeed) // Drive left with negative X (left)
    //         .withRotationalRate(0 * MaxAngularRate) // Drive counterclockwise with negative X (left)
    //     ).ignoringDisable(false);
    // }

    // public void MoveRobotToTargetInFieldCoordinatesWithRotation(double YposeAxis, double XposeAxis, Rotation2d RZposeAxis) {
    //   drivetrain.setControl(fieldpoint.withVelocityX(XposeAxis * MaxSpeedPid) // Drive forward with // negative Y (forward)
    //       .withVelocityY(YposeAxis * MaxSpeedPid) // Drive left with negative X (left)
    //       .withTargetDirection(RZposeAxis) // Drive counterclockwise with negative X (left)
    //   );
    // }
      //

    public final SwerveRequest.FieldCentric FCdriveAuton = new SwerveRequest.FieldCentric();
      //.withDeadband(MaxSpeed * 0.09).withRotationalDeadband(MaxAngularRate * 0.09) // Add a 10% deadband
      //.withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric


  public final SwerveRequest.FieldCentric FCdrive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.09).withRotationalDeadband(MaxAngularRate * 0.09); // Add a 10% deadband
      //.withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  public final SwerveRequest.RobotCentric RobotCentricdrive = new SwerveRequest.RobotCentric()
  //.withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
  .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric


  //private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  //private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  //private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  public final SwerveRequest.FieldCentricFacingAngle fieldpoint = new SwerveRequest.FieldCentricFacingAngle();
}
