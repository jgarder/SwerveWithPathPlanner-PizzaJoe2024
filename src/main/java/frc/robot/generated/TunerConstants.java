package frc.robot.generated;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;

import edu.wpi.first.math.util.Units;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Constants;

public class TunerConstants {
    // Both sets of gains need to be tuned to your individual robot.

    // The steer motor uses any SwerveModule.SteerRequestType control request with the
    // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
    private static final Slot0Configs steerGains = new Slot0Configs()
        .withKP(100).withKI(0).withKD(0.05)
        .withKS(0).withKV(1.5).withKA(0);
    // When using closed-loop control, the drive motor uses the control
    // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
    private static final Slot0Configs driveGains = new Slot0Configs()
        .withKP(3).withKI(0).withKD(0)
        .withKS(0).withKV(0).withKA(0);

    // The closed-loop output type to use for the steer motors;
    // This affects the PID/FF gains for the steer motors
    private static final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;
    // The closed-loop output type to use for the drive motors;
    // This affects the PID/FF gains for the drive motors
    private static final ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;

    // The stator current at which the wheels start to slip;
    // This needs to be tuned to your individual robot
    private static final double kSlipCurrentA = 30.0;

    // Theoretical free speed (m/s) at 12v applied output;
    // This needs to be tuned to your individual robot
    public static final double kSpeedAt12VoltsMps = 4.572;//5.0;//6.0;//5.0;

    // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
    // This may need to be tuned to your individual robot
    private static final double kCoupleRatio = 3.5;

    private static final double kDriveGearRatio = 6.75;//8.14;//7.363636364;
    private static final double kSteerGearRatio = 15.42857143;
    private static final double kWheelRadiusInches = 2.167; // Estimated at first, then fudge-factored to make odom match record

    private static final boolean kSteerMotorReversed = true;
    private static final boolean kInvertLeftSide = false;
    private static final boolean kInvertRightSide = true;

    private static final String kCANbusName = Constants.CANBus.kCANbusName;
    private static final int kPigeonId = Constants.CANBus.kPigeonCanId;


    // These are only used for simulation
    private static final double kSteerInertia = 0.00001;
    private static final double kDriveInertia = 0.001;
    // Simulated voltage necessary to overcome friction
    private static final double kSteerFrictionVoltage = 0.25;
    private static final double kDriveFrictionVoltage = 0.25;

    private static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
            .withPigeon2Id(kPigeonId)
            .withCANbusName(kCANbusName);

    private static final SwerveModuleConstantsFactory ConstantCreator = new SwerveModuleConstantsFactory()
            .withDriveMotorGearRatio(kDriveGearRatio)
            .withSteerMotorGearRatio(kSteerGearRatio)
            .withWheelRadius(kWheelRadiusInches)
            .withSlipCurrent(kSlipCurrentA)
            .withSteerMotorGains(steerGains)
            .withDriveMotorGains(driveGains)
            .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
            .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
            .withSpeedAt12VoltsMps(kSpeedAt12VoltsMps)
            .withSteerInertia(kSteerInertia)
            .withDriveInertia(kDriveInertia)
            .withSteerFrictionVoltage(kSteerFrictionVoltage)
            .withDriveFrictionVoltage(kDriveFrictionVoltage)
            .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
            .withCouplingGearRatio(kCoupleRatio)
            .withSteerMotorInverted(kSteerMotorReversed);


     // Front Left
     private static final int kFrontLeftDriveMotorId = Constants.CANBus.kFrontLeftDriveMotorId;
     private static final int kFrontLeftSteerMotorId = Constants.CANBus.kFrontLeftSteerMotorId;
     private static final int kFrontLeftEncoderId = Constants.CANBus.kFrontLeftEncoderId;
     private static final double kFrontLeftEncoderOffset = 0.070556640625;
 
     private static final double kFrontLeftXPosInches = 17.0625;
     private static final double kFrontLeftYPosInches = 17.0625;
 
     // Front Right
     private static final int kFrontRightDriveMotorId = Constants.CANBus.kFrontRightDriveMotorId;
     private static final int kFrontRightSteerMotorId = Constants.CANBus.kFrontRightSteerMotorId;
     private static final int kFrontRightEncoderId = Constants.CANBus.kFrontRightEncoderId;
     private static final double kFrontRightEncoderOffset = -0.43115234375;
 
     private static final double kFrontRightXPosInches = 17.0625;
     private static final double kFrontRightYPosInches = -17.0625;
 
     // Back Left
     private static final int kBackLeftDriveMotorId = Constants.CANBus.kBackLeftDriveMotorId;
     private static final int kBackLeftSteerMotorId = Constants.CANBus.kBackLeftSteerMotorId;
     private static final int kBackLeftEncoderId = Constants.CANBus.kBackLeftEncoderId;
     private static final double kBackLeftEncoderOffset = 0.102783203125;
 
     private static final double kBackLeftXPosInches = -17.0625;
     private static final double kBackLeftYPosInches = 17.0625;
 
     // Back Right
     private static final int kBackRightDriveMotorId = Constants.CANBus.kBackRightDriveMotorId;
     private static final int kBackRightSteerMotorId = Constants.CANBus.kBackRightSteerMotorId;
     private static final int kBackRightEncoderId = Constants.CANBus.kBackRightEncoderId;
     private static final double kBackRightEncoderOffset = 0.411376953125;
 
     private static final double kBackRightXPosInches = -17.0625;
     private static final double kBackRightYPosInches = -17.0625;


    private static final SwerveModuleConstants FrontLeft = ConstantCreator.createModuleConstants(
            kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId, kFrontLeftEncoderOffset, Units.inchesToMeters(kFrontLeftXPosInches), Units.inchesToMeters(kFrontLeftYPosInches), kInvertLeftSide);
    private static final SwerveModuleConstants FrontRight = ConstantCreator.createModuleConstants(
            kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId, kFrontRightEncoderOffset, Units.inchesToMeters(kFrontRightXPosInches), Units.inchesToMeters(kFrontRightYPosInches), kInvertRightSide);
    private static final SwerveModuleConstants BackLeft = ConstantCreator.createModuleConstants(
            kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId, kBackLeftEncoderOffset, Units.inchesToMeters(kBackLeftXPosInches), Units.inchesToMeters(kBackLeftYPosInches), kInvertLeftSide);
    private static final SwerveModuleConstants BackRight = ConstantCreator.createModuleConstants(
            kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId, kBackRightEncoderOffset, Units.inchesToMeters(kBackRightXPosInches), Units.inchesToMeters(kBackRightYPosInches), kInvertRightSide);

    public static final CommandSwerveDrivetrain DriveTrain = new CommandSwerveDrivetrain(DrivetrainConstants, FrontLeft,
            FrontRight, BackLeft, BackRight);
}
