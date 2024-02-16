// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.MovePickupToPosition;
import frc.robot.commands.RunIntake;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CANdleSystem;
import frc.robot.subsystems.DeliveryLifter;
import frc.robot.subsystems.DeliveryShooter;
import frc.robot.subsystems.DeliveryTilt;
import frc.robot.subsystems.PickupArm;
import frc.robot.subsystems.PickupSpinner;
import frc.robot.subsystems.PickupArm.PickupState;

public class RobotContainer {

  //
  private final PickupArm pickuparm = new PickupArm();
  private final PickupSpinner pickupSpinner = new PickupSpinner();
  private final DeliveryLifter deliveryLifter = new DeliveryLifter();
  private final DeliveryTilt deliveryTilt = new DeliveryTilt();
  private final DeliveryShooter deliveryShooter = new DeliveryShooter();
  //

  public static class PizzaManager{
    public static PickupState LastKnownPickupState = PickupState.ZERO;
    public static boolean IsNoteInPickup = false;
    

  }


  //
  private double MaxSpeed = 6; // 6 meters per second desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  /* Path follower */
  private Command runAuto = drivetrain.getAutoPath("Tests");

  private final Telemetry logger = new Telemetry(MaxSpeed);
  private final CANdleSystem m_candleSubsystem = new CANdleSystem();

  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
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



    joystick.x().onTrue(new InstantCommand(()->{pickupSpinner.ReleaseNote();},pickupSpinner));// (pickuparm.runonce(() -> {pickuparm.setSetpointFloorPickup();}));
    //joystick.y().whileTrue(new InstantCommand(()->{pickuparm.setSetpointFloorPickup();},pickuparm).andThen(()->{pickupSpinner.RunPickup();}).repeatedly()).onFalse(new InstantCommand(()->{pickuparm.setSetpointVerticle();}).andThen(()->{pickupSpinner.HoldAutoLoaded();})); //.until(()->{pickupSpinner.m_forwardLimit.isPressed();})
    joystick.y().onTrue(
    new MovePickupToPosition(Constants.PickupHead.PickupFloorPickup, pickuparm, pickupSpinner)
    .andThen(new InstantCommand(()->{pickupSpinner.setIsnoteInPickup(false);},pickupSpinner))
    .andThen(new RunIntake(pickupSpinner))
    .andThen(new InstantCommand(()->{m_candleSubsystem.GreenLights();},m_candleSubsystem))
    .andThen(new MovePickupToPosition(Constants.PickupHead.PickupPassing, pickuparm, pickupSpinner))
    .andThen(new InstantCommand(()->{pickupSpinner.ReleaseNote();},pickupSpinner))
    .andThen(new InstantCommand(()->{m_candleSubsystem.RedLights();},m_candleSubsystem))
    );
    //left trigger will bring joe into the speaker position
    joystick.leftTrigger().onTrue(new InstantCommand(()->{deliveryLifter.setSetpointZero();},deliveryLifter));
    //left bumper will bring joe into the amp position 
    joystick.leftBumper().onTrue(new InstantCommand(()->{deliveryLifter.setSetpointAmp();},deliveryLifter));
    joystick.pov(0).onTrue(new InstantCommand(() -> {m_candleSubsystem.GreenLights();}));
    joystick.pov(180).onTrue(new InstantCommand(() -> {m_candleSubsystem.RainbowRoadLights();}));
    
    //if in Amp positions or Shoot position, pressing A will execute that action. 
    //joystick.a().whileTrue(runAuto);

    joystick.rightTrigger().onTrue(new InstantCommand(()->{deliveryTilt.setSetpointZero();},deliveryTilt));
    joystick.rightBumper().onTrue(new InstantCommand(()->{deliveryTilt.setSetpointPassing();},deliveryTilt));
  }

  public RobotContainer() {
    configureBindings();
  }

  public Command getAutonomousCommand() {
    /* First put the drivetrain into auto run mode, then run the auto */
    return runAuto;
  }
}
