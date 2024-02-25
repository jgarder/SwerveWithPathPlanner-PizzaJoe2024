// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

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
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ChainLifter;
import frc.robot.RobotContainer.PizzaManager.PizzaTracker;
import frc.robot.commands.AlignSpeakerCMD;
import frc.robot.commands.AlignSpeakerTest;
import frc.robot.commands.MoveChainLiftToPosition;
import frc.robot.commands.MovePickupToPosition;
import frc.robot.commands.RunDeliveryHoldIntake;
import frc.robot.commands.RunIntake;
import frc.robot.commands.ShootDeliveryHold;
import frc.robot.commands.SpoolPizzaDeliveryToRPM;
import frc.robot.commands.UndoDeliveryHold;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CANdleSystem;
import frc.robot.subsystems.ChainLifterS;
import frc.robot.subsystems.DeliveryHolder;
import frc.robot.subsystems.DeliveryLifter;
import frc.robot.subsystems.DeliveryShooter;
import frc.robot.subsystems.DeliveryTilt;
import frc.robot.subsystems.DrivetrainManager;
import frc.robot.subsystems.Limelight3Subsystem;
import frc.robot.subsystems.PickupArm;
import frc.robot.subsystems.PickupSpinner;
import frc.robot.subsystems.SmartDashboardHandler;
import frc.robot.subsystems.PickupArm.PickupState;

public class RobotContainer {



  public final PowerDistribution m_pdh = new PowerDistribution(Constants.CANBus.PDH_CAN_ID,ModuleType.kRev);

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
  public final ChainLifterS ChainLift = new ChainLifterS();
  public final Limelight3Subsystem LL3 = new Limelight3Subsystem(joystick);

  public final SmartDashboardHandler SDashBoardH = new SmartDashboardHandler(this);
  //
  BooleanSupplier isNoteInIntakeboolSup = () -> pickupSpinner.m_forwardLimit.isPressed(); 
  public static class PizzaManager{
    public static PickupState LastKnownPickupState = PickupState.ZERO;
    public static boolean IsNoteInPickup = false;
    public static boolean NoteInDeliveryHolder = false;
    public static boolean AltControlModeEnabled = false;
    public static PizzaTracker pizzaStage = PizzaTracker.Startup; //0-10 empty, 20 detected, 30 passing, 40 passed to head, 50 index started, 51 indexout, 52 Indexed ready to fire, 
    
    public enum PizzaTracker
      {
        Startup,
        empty,
        detected,
        passing,
        passed,
        indexNeeded,
        NoteOutDexed,
        Indexed,
        TrapReady,
      }
    
    public boolean IsAltControlModeEnabled()
    {
      return AltControlModeEnabled;
    }
    public boolean IsNoteInDeliveryHolder()
    {
      return NoteInDeliveryHolder;
    }

    
  }




  

  
  private final CANdleSystem m_candleSubsystem = new CANdleSystem();

  public Command C_PickupPizzaFromFloorWithoutWashing()
  {
    return //new RunDeliveryHoldIntake(deliveryHolder).alongWith(
    new InstantCommand(()->{pickupSpinner.setIsnoteInPickup(false);},pickupSpinner)
    .andThen(new RunIntake(pickupSpinner).withInterruptBehavior(InterruptionBehavior.kCancelIncoming))
    .andThen(new InstantCommand(()->{intakepassing = false;}))
    .andThen(C_ReturnPickupHead().alongWith(new InstantCommand(()->{pickupSpinner.IntakeRunCommand(5);},pickupSpinner)
    .andThen(new WaitCommand(.5))
    .andThen(new InstantCommand(()->{pickupSpinner.stopSpinner();}))))//POST ROLL PICKUP INTAKE SPIN)
    
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
  public boolean intakepassing = false;
  public Command C_ReturnPickupToPassing()
  {
    // if (intakepassing) {
    //   return new InstantCommand(()->{System.out.println("currently passing to the pickup");});
    // }
    intakepassing = true;
    return C_ReturnPickupHead().andThen(C_CatchAndIndexNote().withInterruptBehavior(InterruptionBehavior.kCancelIncoming).alongWith(C_passNoteFromIntakeToDeliveryHolder())
    
    );
  }
  public Command C_passNoteFromIntakeToDeliveryHolder()
  {
 return  //small wait for debounce maybe take out and make grab deeper.
    new InstantCommand(()->{pickupSpinner.ReleaseNote();},pickupSpinner)
    .andThen(new WaitCommand(.85))//THIS WAIT COMMAND IS THE POST ROLL OUTPUT PASSING ROLL //.onlyWhile((pickupSpinner::getLimitSwitchEnabled))
    .andThen(new InstantCommand(()->{pickupSpinner.stopSpinner();}))
    .andThen(new InstantCommand(()->{intakepassing = false;}));
  }
  public Command C_CatchAndIndexNote()
  {
     return new InstantCommand(()->{deliveryHolder.RequestIndex();})
     .alongWith(new InstantCommand(()->{PizzaManager.pizzaStage = PizzaTracker.passed;})
     );
      // return new SequentialCommandGroup(
      //   new RunDeliveryHoldIntake(deliveryHolder,false,40).withInterruptBehavior(InterruptionBehavior.kCancelIncoming),
      //   new UndoDeliveryHold(deliveryHolder).withInterruptBehavior(InterruptionBehavior.kCancelIncoming),
      //   new RunDeliveryHoldIntake(deliveryHolder,false,30).withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
      //   ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
  }
  public Command C_ReturnPickupHead()
  {

     return new MovePickupToPosition(Constants.PickupHead.PickupPassing, pickuparm)
    .andThen(new WaitCommand(.175)) //small wait for debounce maybe take out and make grab deeper.
    ;
  }

  public Command C_ParkDeliveryHead()
  {
    return  new InstantCommand(()->{deliveryLifter.setSetpointZero();},deliveryLifter)
      .alongWith(C_TiltGotoPark(),new InstantCommand(()->deliveryShooter.SetShootSpeed(Constants.DeliveryHead.ShooterRpmOff)));
  }

  public Command C_TiltGotoPark()
  {
    return new InstantCommand(()->{deliveryTilt.setSetpointToPosition(Constants.DeliveryHead.Tilt_Position_Passing);},deliveryTilt).andThen(new WaitCommand(20).until(()->deliveryLifter.CurrentLiftEncoderValue < 20)).andThen(new InstantCommand(()->{deliveryTilt.setSetpointToPosition(Constants.DeliveryHead.Tilt_Position_Park);},deliveryTilt));
  }
  public Command C_ReadyCloseSpeakerShot()
  {
    return new InstantCommand(()->{deliveryLifter.setSetpointPassing();},deliveryLifter)
      .alongWith(new InstantCommand(()->{deliveryTilt.setSetpointToPosition(Constants.DeliveryHead.Tilt_Position_Speaker_Closest);},deliveryTilt),new SpoolPizzaDeliveryToRPM(deliveryShooter, Constants.DeliveryHead.ShooterRpmSpeakerClose));
  }
  private void configureBindings() {
    drivetrainManager.configureBindings();
    joystick.back().onTrue(new InstantCommand(()->{PizzaManager.AltControlModeEnabled = !PizzaManager.AltControlModeEnabled;}));
    
    joystick.rightBumper()
    .onTrue(new SequentialCommandGroup(new MovePickupToPosition(Constants.PickupHead.PickupFloorPickup, pickuparm).unless(isNoteInIntakeboolSup),C_PickupPizzaFromFloorWithoutWashing().unless(isNoteInIntakeboolSup),C_ReturnPickupToPassing()))
    .onFalse(C_ReturnPickupHead().unless(isNoteInIntakeboolSup).andThen(new InstantCommand(()->{pickupSpinner.stopSpinner();})));//.andThen(C_passNoteFromIntakeToDeliveryHolder().alongWith(C_CatchAndIndexNote().withInterruptBehavior(InterruptionBehavior.kCancelSelf)).onlyIf(isNoteInIntakeboolSup)).andThen(new InstantCommand(()->{pickupSpinner.stopSpinner();})));

    joystick.a().onTrue(C_CatchAndIndexNote());
    joystick.b().onTrue(new ShootDeliveryHold(deliveryHolder));
    joystick.x().onTrue(new InstantCommand(()->{pickupSpinner.ReleaseNote();},pickupSpinner).andThen(new RunDeliveryHoldIntake(deliveryHolder,false,40)))
    .onFalse(new InstantCommand(()->{pickupSpinner.stopSpinner();},pickupSpinner));// (pickuparm.runonce(() -> {pickuparm.setSetpointFloorPickup();}));
    //joystick.y().whileTrue(new InstantCommand(()->{pickuparm.setSetpointFloorPickup();},pickuparm).andThen(()->{pickupSpinner.RunPickup();}).repeatedly()).onFalse(new InstantCommand(()->{pickuparm.setSetpointVerticle();}).andThen(()->{pickupSpinner.HoldAutoLoaded();})); //.until(()->{pickupSpinner.m_forwardLimit.isPressed();})
    //joystick.y().onTrue(C_PickupPizzaFromFloorWithoutWashing());

    //left trigger will bring joe into the speaker position
    joystick.leftTrigger().whileTrue(
      C_ReadyCloseSpeakerShot().onlyIf(()->PizzaManager.NoteInDeliveryHolder))
      .onFalse(C_ParkDeliveryHead());

//left trigger will bring joe into the speaker position
    joystick.rightTrigger().whileTrue(
      new InstantCommand(()->{deliveryLifter.setSetpointZero();},deliveryLifter)
      .alongWith(new InstantCommand(()->{deliveryTilt.setSetpointToPosition(Constants.DeliveryHead.Tilt_Position_Speaker_SafePost);},deliveryTilt),new InstantCommand(()->deliveryShooter.SetShootSpeed(Constants.DeliveryHead.ShooterRpmSpeakerClose))).onlyIf(()->PizzaManager.NoteInDeliveryHolder))
      .onFalse(C_ParkDeliveryHead());




    //left bumper will bring joe into the amp position 
    joystick.leftBumper().whileTrue(
      new InstantCommand(()->{deliveryLifter.setSetpointAmp();},deliveryLifter)
            .alongWith(new InstantCommand(()->{deliveryTilt.setSetpointToPosition(Constants.DeliveryHead.Tilt_Position_Amp);},deliveryTilt),new InstantCommand(()->deliveryShooter.SetShootSpeed(Constants.DeliveryHead.ShooterRpmAmp))).onlyIf(()->PizzaManager.NoteInDeliveryHolder))
            .onFalse(C_ParkDeliveryHead());
    int strafeAxis = 4;
    //DoubleSupplier sup = () -> joystick.getRawAxis(strafeAxis);

    joystick.pov(Constants.XboxControllerMap.kPOVDirectionLeft).and(()->!PizzaManager.AltControlModeEnabled)
    .whileTrue(new AlignSpeakerCMD(drivetrainManager,LL3,() -> joystick.getRawAxis(strafeAxis))
    .andThen(
      C_ReadyCloseSpeakerShot(),
      //new WaitCommand(1),
    (new ShootDeliveryHold(deliveryHolder)),
    C_ParkDeliveryHead()))
    .onFalse(C_ParkDeliveryHead());

    joystick.pov(Constants.XboxControllerMap.kPOVDirectionRIGHT).and(()->!PizzaManager.AltControlModeEnabled)
    .whileTrue(new InstantCommand(()->{m_candleSubsystem.RainbowRoadLights();})).onFalse(new InstantCommand(()->{m_candleSubsystem.GreenLights();}));
      //  joystick.pov(Constants.XboxControllerMap.kPOVDirectionRIGHT).and(()->!PizzaManager.AltControlModeEnabled)
      //  .whileTrue(
      // new InstantCommand(()->{deliveryLifter.setSetpoint(Constants.DeliveryHead.Lift_Position_Trap);},deliveryLifter)
      // .alongWith(new InstantCommand(()->{deliveryTilt.setSetpointToPosition(Constants.DeliveryHead.Tilt_Position_Amp);},deliveryTilt)))
      //  .onFalse(new InstantCommand(()->{deliveryLifter.setSetpointZero();},deliveryLifter).alongWith(C_TiltGotoPark()));
      //.whileTrue(new AlignSpeakerTest(drivetrainManager,LL3));
    
    joystick.pov(Constants.XboxControllerMap.kPOVDirectionUP).and(()->!PizzaManager.AltControlModeEnabled)
    .whileTrue(new InstantCommand(()->{drivetrainManager.move(0,0,0);}));
    joystick.pov(Constants.XboxControllerMap.kPOVDirectionDOWN).and(()->!PizzaManager.AltControlModeEnabled)
    .whileTrue(drivetrainManager.drivetrain.applyRequest(() -> drivetrainManager.drive.withVelocityX(-.1 * drivetrainManager.MaxSpeed) // Drive forward with                                                                                        // negative Y (forward)
            .withVelocityY(-.1 * drivetrainManager.MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(0 * drivetrainManager.MaxAngularRate) // Drive counterclockwise with negative X (left)
        ).ignoringDisable(false)).onFalse(
          drivetrainManager.drivetrain.applyRequest(() -> drivetrainManager.drive.withVelocityX(0 * drivetrainManager.MaxSpeed) // Drive forward with                                                                                        // negative Y (forward)
            .withVelocityY(0 * drivetrainManager.MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(0 * drivetrainManager.MaxAngularRate) // Drive counterclockwise with negative X (left)
        ).ignoringDisable(false)
        );
     //joystick.pov(Constants.XboxControllerMap.kPOVDirectionRIGHT).onTrue(new InstantCommand(() -> {deliveryTilt.AlterSetpointposition(-.2);}));
    //if in Amp positions or Shoot position, pressing A will execute that action. 
    //joystick.a().whileTrue(runAuto);

    //joystick.rightTrigger().onTrue(new InstantCommand(()->{deliveryTilt.setSetpointZero();},deliveryTilt));
    
    //.onTrue(new InstantCommand(()->{deliveryTilt.setSetpointPassing();},deliveryTilt));
    configureTrapButtons();
  }



  public void configureTrapButtons()
  {
    //going DOWN
    joystick.pov(Constants.XboxControllerMap.kPOVDirectionDOWN).and(()->PizzaManager.AltControlModeEnabled)
    .onTrue(new MoveChainLiftToPosition(Constants.ChainLifter.Lift_Position_Zero, ChainLift).alongWith(new InstantCommand(()->{deliveryTilt.setSetpointToPosition(Constants.DeliveryHead.Tilt_Position_Park);}))
    .andThen(new MovePickupToPosition(Constants.PickupHead.PickupPassing, pickuparm)
    .alongWith(C_TiltGotoPark())
    ));
    
    joystick.pov(Constants.XboxControllerMap.kPOVDirectionRIGHT).and(()->PizzaManager.AltControlModeEnabled)
    .onTrue(new MoveChainLiftToPosition(Constants.ChainLifter.Lift_Position_PullDown, ChainLift)
      .alongWith(new InstantCommand(()->{deliveryTilt.setSetpointToPosition(Constants.DeliveryHead.Tilt_Position_TrapLiftUp);})
      .andThen(new WaitCommand(1.0))
      .andThen(new MovePickupToPosition(Constants.PickupHead.PickupFloorPickup, pickuparm))
      .andThen(new InstantCommand(()->{deliveryLifter.setSetpoint(Constants.DeliveryHead.Lift_Position_Trap);},deliveryLifter))
      .andThen(new WaitCommand(2.0))
      .andThen(new MovePickupToPosition(Constants.PickupHead.PickupVertical, pickuparm)))
      );


    joystick.pov(Constants.XboxControllerMap.kPOVDirectionUP).and(()->PizzaManager.AltControlModeEnabled)
    .onTrue(new MovePickupToPosition(Constants.PickupHead.PickupVertical, pickuparm)
    .alongWith(new InstantCommand(()->{deliveryLifter.setSetpoint(Constants.DeliveryHead.Lift_Position_TrapStart);},deliveryLifter))
    .alongWith(new InstantCommand(()->{deliveryTilt.setSetpointToPosition(Constants.DeliveryHead.Tilt_Position_TrapLift);}))
    .andThen(new MoveChainLiftToPosition(Constants.ChainLifter.Lift_Position_CenterAndTrap, ChainLift)));
    
    joystick.pov(Constants.XboxControllerMap.kPOVDirectionLeft).and(()->PizzaManager.AltControlModeEnabled)
    .onTrue(new InstantCommand(() -> {deliveryTilt.setSetpointToPosition(Constants.DeliveryHead.Tilt_Position_TrapLiftUpSHOOT);})
    .alongWith(new InstantCommand(()->{deliveryLifter.setSetpoint(Constants.DeliveryHead.Lift_Position_TrapShoot);},deliveryLifter))
    .alongWith(new MovePickupToPosition(Constants.PickupHead.PickupFloorPickup, pickuparm).alongWith(new InstantCommand(()->deliveryShooter.SetShootSpeed(Constants.DeliveryHead.ShooterRpmSpeakerClose))))
    )
    .onFalse(new InstantCommand(()->deliveryShooter.SetShootSpeed(Constants.DeliveryHead.ShooterRpmOff))
    .alongWith(new InstantCommand(() -> {deliveryTilt.setSetpointToPosition(Constants.DeliveryHead.Tilt_Position_TrapLiftUp);}).andThen(new MovePickupToPosition(Constants.PickupHead.PickupVertical, pickuparm))));
    //////
  }
  public RobotContainer() {
    configureBindings();
  }

  public Command getAutonomousCommand() {
    /* First put the drivetrain into auto run mode, then run the auto */
    return drivetrainManager.runAuto;
  }
  

}
