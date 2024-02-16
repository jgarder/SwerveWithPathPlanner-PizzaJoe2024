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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.MovePickupToPosition;
import frc.robot.commands.RunIntake;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CANdleSystem;
import frc.robot.subsystems.DeliveryLifter;
import frc.robot.subsystems.DeliveryShooter;
import frc.robot.subsystems.DeliveryTilt;
import frc.robot.subsystems.DrivetrainManager;
import frc.robot.subsystems.PickupArm;
import frc.robot.subsystems.PickupSpinner;
import frc.robot.subsystems.PickupArm.PickupState;

public class RobotContainer {

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  
  //
  private final PickupArm pickuparm = new PickupArm();
  private final PickupSpinner pickupSpinner = new PickupSpinner();
  private final DeliveryLifter deliveryLifter = new DeliveryLifter();
  private final DeliveryTilt deliveryTilt = new DeliveryTilt();
  private final DeliveryShooter deliveryShooter = new DeliveryShooter();
  public final DrivetrainManager drivetrainManager = new DrivetrainManager(joystick);
  //

  public static class PizzaManager{
    public static PickupState LastKnownPickupState = PickupState.ZERO;
    public static boolean IsNoteInPickup = false;
    

  }




  

  
  private final CANdleSystem m_candleSubsystem = new CANdleSystem();

  private void configureBindings() {
    drivetrainManager.configureBindings();

    



    joystick.x().onTrue(new InstantCommand(()->{pickupSpinner.ReleaseNotecommand();},pickupSpinner));// (pickuparm.runonce(() -> {pickuparm.setSetpointFloorPickup();}));
    //joystick.y().whileTrue(new InstantCommand(()->{pickuparm.setSetpointFloorPickup();},pickuparm).andThen(()->{pickupSpinner.RunPickup();}).repeatedly()).onFalse(new InstantCommand(()->{pickuparm.setSetpointVerticle();}).andThen(()->{pickupSpinner.HoldAutoLoaded();})); //.until(()->{pickupSpinner.m_forwardLimit.isPressed();})
    joystick.y().onTrue(
    new MovePickupToPosition(Constants.PickupHead.PickupFloorPickup, pickuparm)
    .andThen(new InstantCommand(()->{pickupSpinner.setIsnoteInPickup(false);},pickupSpinner))
    .andThen(new RunIntake(pickupSpinner))
    .andThen(new InstantCommand(()->{m_candleSubsystem.GreenLights();},m_candleSubsystem))
    .andThen(new MovePickupToPosition(Constants.PickupHead.PickupPassing, pickuparm))
    .andThen(new InstantCommand(()->{pickupSpinner.ReleaseNotecommand();},pickupSpinner))
    .andThen(new InstantCommand(()->{m_candleSubsystem.RedLights();},m_candleSubsystem))
    .andThen(new WaitCommand(1))
    .andThen(new InstantCommand(()->{pickupSpinner.disable();}))
    );
    //left trigger will bring joe into the speaker position
    joystick.leftTrigger().onTrue(new InstantCommand(()->{deliveryLifter.setSetpointZero();},deliveryLifter));
    //left bumper will bring joe into the amp position 
    joystick.leftBumper().onTrue(new InstantCommand(()->{deliveryLifter.setSetpointPassing();},deliveryLifter));
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
    return drivetrainManager.runAuto;
  }
}
