// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.MovePickupToPosition;
import frc.robot.commands.RunDeliveryHoldIntake;
import frc.robot.commands.RunIntake;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CANdleSystem;
import frc.robot.subsystems.DeliveryHolder;
import frc.robot.subsystems.DeliveryLifter;
import frc.robot.subsystems.DeliveryShooter;
import frc.robot.subsystems.DeliveryTilt;
import frc.robot.subsystems.DrivetrainManager;
import frc.robot.subsystems.PickupArm;
import frc.robot.subsystems.PickupSpinner;
import frc.robot.subsystems.SmartDashboardHandler;
import frc.robot.subsystems.PickupArm.PickupState;

public class RobotContainer {

  private static final int PDH_CAN_ID = 1;
  private static final int NUM_PDH_CHANNELS =24;

  PowerDistribution m_pdh = new PowerDistribution(PDH_CAN_ID,ModuleType.kRev);

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  
  //
  public final PickupArm pickuparm = new PickupArm();
  public final PickupSpinner pickupSpinner = new PickupSpinner();
  public final DeliveryLifter deliveryLifter = new DeliveryLifter();
  public final DeliveryTilt deliveryTilt = new DeliveryTilt();
  public final DeliveryShooter deliveryShooter = new DeliveryShooter();
  public final DrivetrainManager drivetrainManager = new DrivetrainManager(joystick);
  public final DeliveryHolder deliveryHolder = new DeliveryHolder();


  public final SmartDashboardHandler SDashBoardH = new SmartDashboardHandler(this);
  //

  public static class PizzaManager{
    public static PickupState LastKnownPickupState = PickupState.ZERO;
    public static boolean IsNoteInPickup = false;
    public static boolean IsNoteInDeliveryPickup = false;
    

  }




  

  
  private final CANdleSystem m_candleSubsystem = new CANdleSystem();

  public Command C_PickupPizzaFromFloorWithoutWashing()
  {
    return //new RunDeliveryHoldIntake(deliveryHolder).alongWith(
    new MovePickupToPosition(Constants.PickupHead.PickupFloorPickup, pickuparm)
    .andThen(new InstantCommand(()->{pickupSpinner.setIsnoteInPickup(false);},pickupSpinner))
    .andThen(new RunIntake(pickupSpinner)).andThen(new InstantCommand(()->{pickupSpinner.IntakeRunCommand(25);},pickupSpinner))
    .andThen(new InstantCommand(()->{m_candleSubsystem.GreenLights();},m_candleSubsystem))
    // .andThen(new MovePickupToPosition(Constants.PickupHead.PickupPassing, pickuparm))
    //   //.alongWith(new InstantCommand(()->{pickupSpinner.IntakeRunCommand(50);}))
    //     //.alongWith(new RunDeliveryHoldIntake(deliveryHolder)))
    
    // .andThen(new InstantCommand(()->{pickupSpinner.ReleaseNote();},pickupSpinner))
    // .andThen(new InstantCommand(()->{m_candleSubsystem.RedLights();},m_candleSubsystem))
    // .andThen(new WaitCommand(2))
    // .andThen(new InstantCommand(()->{pickupSpinner.disable();}))
    // .andThen(new WaitCommand(3))
    
    //)
    ;
  }
  public Command C_ReturnPickupToPassing()
  {
    return new RunDeliveryHoldIntake(deliveryHolder,false).alongWith(
    new MovePickupToPosition(Constants.PickupHead.PickupPassing, pickuparm).andThen(new WaitCommand(.125)) //small wait for debounce maybe take out and make grab deeper.
    .andThen(new InstantCommand(()->{pickupSpinner.ReleaseNote();},pickupSpinner))
    .andThen(new WaitCommand(1.25))//.onlyWhile((pickupSpinner::getLimitSwitchEnabled))
    .andThen(new InstantCommand(()->{pickupSpinner.stopSpinner();}))
    )
    ;
  }

  public Command C_ParkDeliveryHead()
  {
    return  new InstantCommand(()->{deliveryLifter.setSetpointZero();},deliveryLifter)
      .alongWith(new InstantCommand(()->{deliveryTilt.setSetpointToPosition(Constants.DeliveryHead.Tilt_Position_Park);},deliveryTilt),new InstantCommand(()->deliveryShooter.SetShootSpeed(Constants.DeliveryHead.ShooterRpmOff)));
  }

  private void configureBindings() {
    drivetrainManager.configureBindings();

    joystick.rightBumper().whileTrue(C_PickupPizzaFromFloorWithoutWashing().andThen(C_ReturnPickupToPassing())).onFalse(C_ReturnPickupToPassing());

    
    joystick.b().onTrue((new RunDeliveryHoldIntake(deliveryHolder,true)).withTimeout(2));
    joystick.x().onTrue(new InstantCommand(()->{pickupSpinner.ReleaseNote();},pickupSpinner))
    .onFalse(new InstantCommand(()->{pickupSpinner.stopSpinner();},pickupSpinner));// (pickuparm.runonce(() -> {pickuparm.setSetpointFloorPickup();}));
    //joystick.y().whileTrue(new InstantCommand(()->{pickuparm.setSetpointFloorPickup();},pickuparm).andThen(()->{pickupSpinner.RunPickup();}).repeatedly()).onFalse(new InstantCommand(()->{pickuparm.setSetpointVerticle();}).andThen(()->{pickupSpinner.HoldAutoLoaded();})); //.until(()->{pickupSpinner.m_forwardLimit.isPressed();})
    //joystick.y().onTrue(C_PickupPizzaFromFloorWithoutWashing());

    //left trigger will bring joe into the speaker position
    joystick.leftTrigger().whileTrue(
      new InstantCommand(()->{deliveryLifter.setSetpointPassing();;},deliveryLifter)
      .alongWith(new InstantCommand(()->{deliveryTilt.setSetpointToPosition(Constants.DeliveryHead.Tilt_Position_Speaker_Closest);},deliveryTilt),new InstantCommand(()->deliveryShooter.SetShootSpeed(Constants.DeliveryHead.ShooterRpmSpeakerClose))))
      .onFalse(C_ParkDeliveryHead());
    //left bumper will bring joe into the amp position 
    joystick.leftBumper().whileTrue(
      new InstantCommand(()->{deliveryLifter.setSetpointAmp();},deliveryLifter)
            .alongWith(new InstantCommand(()->{deliveryTilt.setSetpointToPosition(Constants.DeliveryHead.Tilt_Position_Amp);},deliveryTilt),new InstantCommand(()->deliveryShooter.SetShootSpeed(Constants.DeliveryHead.ShooterRpmAmp))))
            .onFalse(C_ParkDeliveryHead());
    
    joystick.pov(Constants.XboxControllerMap.kPOVDirectionDOWN).onTrue(new InstantCommand(() -> {m_candleSubsystem.GreenLights();}));
    joystick.pov(Constants.XboxControllerMap.kPOVDirectionUP).onTrue(new InstantCommand(() -> {m_candleSubsystem.RainbowRoadLights();}));
    
    //if in Amp positions or Shoot position, pressing A will execute that action. 
    //joystick.a().whileTrue(runAuto);

    joystick.rightTrigger().onTrue(new InstantCommand(()->{deliveryTilt.setSetpointZero();},deliveryTilt));
    
    //.onTrue(new InstantCommand(()->{deliveryTilt.setSetpointPassing();},deliveryTilt));
  }

  public RobotContainer() {
    configureBindings();
  }

  public Command getAutonomousCommand() {
    /* First put the drivetrain into auto run mode, then run the auto */
    return drivetrainManager.runAuto;
  }
  private double maxcurrent = 0.0;
  public double lowestVoltage = 48.00;
  public void UpdatePDHToSmartDashboard() {
     /**
     * Get the input voltage of the PDH and display it on Shuffleboard.
     */
    double Voltage = m_pdh.getVoltage();
    if (Voltage < lowestVoltage){
      lowestVoltage = Voltage;
    }
    SmartDashboard.putNumber("Voltage", Voltage);
    SmartDashboard.putNumber("Lowest Voltage", lowestVoltage);
    /**
     * Get the total current of the PDH and display it on Shuffleboard. This will
     * be to the nearest even number.
     *
     * To get a better total current reading, sum the currents of all channels.
     * See below for getting channel current.
     */
    double current = m_pdh.getTotalCurrent();
    if (current > maxcurrent){
      maxcurrent = current;
    }
    SmartDashboard.putNumber("Total Current",current);
    SmartDashboard.putNumber("MAX Total Current", maxcurrent);
    /**
     * Get the currents of each channel of the PDH and display them on
     * Shuffleboard.
     */
    for (int channel = 0; channel < NUM_PDH_CHANNELS; channel++) {
      SmartDashboard.putNumber(
          ("Ch" + String.valueOf(channel) + " Current"),
          m_pdh.getCurrent(channel));
    }
  }

}
