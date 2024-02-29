package frc.robot.subsystems;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Constants;
import frc.robot.Telemetry;
import frc.robot.RobotContainer.PizzaManager;
import frc.robot.generated.TunerConstants;
import com.pathplanner.lib.auto.NamedCommands;

public class DrivetrainManager extends SubsystemBase{
    
    
  public double MaxSpeed = 9;//6; // 6 meters per second desired top speed
  public double MaxAngularRate = 2.5 * Math.PI; //1.8 ==  3/4 of a rotation per second max angular velocity

  public double MaxSpeedPid = 2;//6; // 6 meters per second desired top speed
  public double MaxAngularRatePid = 1.0 * Math.PI; // 3/4 of a rotation per second max angular velocity

  
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
    
    public static final double slewrateXY = 20;//3;
    public static final double slewrateRotation = 20;//2.7;
    SlewRateLimiter speedLimiterX = new SlewRateLimiter(slewrateXY);
    SlewRateLimiter speedLimiterY = new SlewRateLimiter(slewrateXY);
    SlewRateLimiter speedLimiterRotation = new SlewRateLimiter(slewrateRotation);
    public void configureBindings()
    {
        drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> FCdrive.withVelocityX(speedLimiterY.calculate(-joystick.getLeftY()) * (MaxSpeed*PizzaManager.speedMulti)) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(speedLimiterX.calculate(-joystick.getLeftX()) * (MaxSpeed*PizzaManager.speedMulti)) // Drive left with negative X (left)
            .withRotationalRate(speedLimiterRotation.calculate(-joystick.getRightX()) * (MaxAngularRate*PizzaManager.RotationMulti)) // Drive counterclockwise with negative X (left)
        ).ignoringDisable(true));

        //stuff below should be tested when drivetrain is complete    
        //joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        //joystick.b().whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

        //joystick.pov(0).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(0)));
        //joystick.pov(180).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)));
        
        // reset the field-centric heading on left bumper press
        joystick.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

        // if (Utils.isSimulation()) {
        //   drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
        // }
        drivetrain.registerTelemetry(logger::telemeterize);


    }

    public void move(double x,double y,double r)
    {
      drivetrain.applyRequest(() -> FCdrive.withVelocityX(-.1 * MaxSpeed) // Drive forward with                                                                                        // negative Y (forward)
            .withVelocityY(-.1 * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(0 * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ).ignoringDisable(false);
    }
      //




  public final SwerveRequest.FieldCentric FCdrive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.09).withRotationalDeadband(MaxAngularRate * 0.09) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  public final SwerveRequest.RobotCentric RobotCentricdrive = new SwerveRequest.RobotCentric()
  //.withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
  .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric


  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
}
