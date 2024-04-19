// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ChainLifter;
import frc.robot.RobotContainer.PizzaManager.PizzaTracker;
import frc.robot.commands.AlignAmpCMD;
import frc.robot.commands.AlignSourceCMD;
import frc.robot.commands.AlignSpeakerCMD;
import frc.robot.commands.AlignSpeakerCMD;
import frc.robot.commands.AlignSpeakerTest;
import frc.robot.commands.AlignStageCMD;
import frc.robot.commands.AlignTrapShootCMD;
import frc.robot.commands.FlashBlueLights;
import frc.robot.commands.FlashGreenLights;
import frc.robot.commands.FlashRainbowLights;
import frc.robot.commands.ForwardBump;
import frc.robot.commands.MoveChainLiftToPosition;
import frc.robot.commands.MoveDLifterToPosition;
import frc.robot.commands.MoveDTiltToPosition;
import frc.robot.commands.MovePickupToPosition;
import frc.robot.commands.RunDeliveryHoldIntake;
import frc.robot.commands.RunIntake;
import frc.robot.commands.ShootDeliveryHold;
import frc.robot.commands.SpoolPizzaDeliveryToRPM;
import frc.robot.commands.TrapShootRPM;
import frc.robot.commands.TrapShootTILT;
import frc.robot.commands.UndoDeliveryHold;
import frc.robot.commands.WaitForIndexCMD;
import frc.robot.commands.ZeroDtilt;
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


public class RobotContainer {

  int strafeAxis = 4;
  private final SendableChooser<Command> autoChooser;

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
  public final DeliveryHolder deliveryHolder = new DeliveryHolder(deliveryShooter,pickupSpinner,this);
  public final ChainLifterS ChainLift = new ChainLifterS();
  public final Limelight3Subsystem LL3 = new Limelight3Subsystem();
  private final CANdleSystem m_candleSubsystem = new CANdleSystem();
  public final SmartDashboardHandler SDashBoardH = new SmartDashboardHandler(this);
  //
  BooleanSupplier isNoteInIntakeboolSup = () -> pickupSpinner.m_forwardLimit.isPressed(); 
  public BooleanSupplier isNoteInDeliveryHolderboolSup = () -> deliveryHolder.m_forwardLimit.isPressed(); 
  BooleanSupplier IsLimeLightBypassed = () -> PizzaManager.LimeLightBypassed; 
  public static class PizzaManager{
    public static double speedMulti = 1.0;
    public static double RotationMulti = .45;
    public static boolean HeadParked = false;
    public static boolean HasTiltBeenZeroed = false;
    public static boolean LimelightTelemetryUpdateRequested = true;
    public static boolean LimeLightBypassed = false;
    public static boolean IsNoteInPickup = false;
    public static boolean NoteInDeliveryHolder = false;
    public static boolean AltControlModeEnabled = false;
    public static PizzaTracker pizzaStage = PizzaTracker.Startup; //0-10 empty, 20 detected, 30 passing, 40 passed to head, 50 index started, 51 indexout, 52 Indexed ready to fire, 
    public static boolean TrapShotOverIndexed = false;
    public enum PizzaTracker
      {
        Startup,
        empty,
        detected,
        passing,
        passed,
        intaking,
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
    public static BooleanSupplier IsOvenPickUpRunning;// = ()->{joystick.rightTrigger(.1).getAsBoolean();};
    public static boolean RequestTrapLights;
    
  }




  

  
  
 public double intakePostRollSeconds = 0.15;//.4;//0.175;//this is added to auton time :(
  public Command C_PickupPizzaFromFloorWithoutWashing()
  {
    return //new RunDeliveryHoldIntake(deliveryHolder).alongWith(
    new InstantCommand(()->{pickupSpinner.setIsnoteInPickup(false);},pickupSpinner)
    .andThen(new RunIntake(pickupSpinner).withInterruptBehavior(InterruptionBehavior.kCancelIncoming))
    .andThen(new InstantCommand(()->{m_candleSubsystem.StrobeBlueLights();},m_candleSubsystem))
    .andThen(new InstantCommand(()->{intakepassing = false;}))
    .andThen(
      //new InstantCommand(()->{m_candleSubsystem.BlueLights();},m_candleSubsystem),
      C_ReturnPickupHead()
      .alongWith(new InstantCommand(()->{pickupSpinner.IntakePostRollCommand();},pickupSpinner)
      .andThen(new WaitCommand(intakePostRollSeconds))//POST ROLL PICKUP INTAKE SPIN
      .andThen(new InstantCommand(()->{pickupSpinner.stopSpinner();}))
        )
    )//POST ROLL PICKUP INTAKE SPIN)
   
    //.andThen()
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
    return C_ReturnPickupHead()
    .andThen(
      C_CatchAndIndexNote().withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
      .alongWith(C_passNoteFromIntakeToDeliveryHolder())
    );
  }
  public double PostRollOutputPassing = .35;
  public Command C_passNoteFromIntakeToDeliveryHolder()
  {
 return  //small wait for debounce maybe take out and make grab deeper.
    new InstantCommand(()->{pickupSpinner.ReleaseNote();},pickupSpinner)
    .andThen(new WaitCommand(PostRollOutputPassing))//THIS WAIT COMMAND IS THE POST ROLL OUTPUT PASSING ROLL //.onlyWhile((pickupSpinner::getLimitSwitchEnabled))
    .andThen(new InstantCommand(()->{pickupSpinner.stopSpinner();}))
    .andThen(new InstantCommand(()->{intakepassing = false;}));
  }
  public Command C_CatchAndIndexNote()
  {
     return new InstantCommand(()->{deliveryHolder.RequestIndex();})
     .alongWith(new InstantCommand(()->{PizzaManager.pizzaStage = PizzaTracker.passing;})
     );
      // return new SequentialCommandGroup(
      //   new RunDeliveryHoldIntake(deliveryHolder,false,40).withInterruptBehavior(InterruptionBehavior.kCancelIncoming),
      //   new UndoDeliveryHold(deliveryHolder).withInterruptBehavior(InterruptionBehavior.kCancelIncoming),
      //   new RunDeliveryHoldIntake(deliveryHolder,false,30).withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
      //   ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
  }
  public Command C_ReturnPickupHead()
  {

     return new MovePickupToPosition(Constants.PizzaFloorPickupHead.PickupPassing, pickuparm)
    .andThen(new WaitCommand(.05))//.175 //small wait for debounce maybe take out and make grab deeper.
    ;
  }

  public Command C_ParkDeliveryHead()
  {
    return  new InstantCommand(()->{deliveryLifter.setSetpointZero();},deliveryLifter)
      .alongWith(
        C_TiltGotoPark(),
      new InstantCommand(()->deliveryShooter.SetShootSpeed(Constants.DeliveryHead.ShooterRpmOff)))
      .andThen(new InstantCommand(()->{m_candleSubsystem.TimeOffLight();}).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
  }
  public Command ReadyShootPreEmptive()
  {
    return new InstantCommand(()->{ deliveryShooter.SetShootSpeed(Constants.DeliveryHead.ShooterRpmSpeakerKnownClose);});
  }
  public double safeLifterPosition = Constants.DeliveryHead.Lift_Position_ParkingPos;
  //public double debouncetime = .1;
  public Timer debouncetimer = new Timer();
  public Command C_TiltGotoPark()
  {
    return new MoveDTiltToPosition(Constants.DeliveryHead.Tilt_Position_Park, deliveryTilt).alongWith(new InstantCommand(()->{debouncetimer.restart();}))
    .until(()->deliveryLifter.CurrentLiftEncoderValue < safeLifterPosition | deliveryTilt.CurrentEncoderValue <= Constants.DeliveryHead.Tilt_Position_Park)
    .andThen(new WaitCommand(8)
    .until(()->deliveryLifter.CurrentLiftEncoderValue < safeLifterPosition))
    .andThen(
      new InstantCommand(()->{deliveryTilt.setSetpointToPosition(Constants.DeliveryHead.Tilt_Position_Zero);},deliveryTilt)//, 
      //new WaitCommand(1.00)//,
    //new InstantCommand(()->{deliveryTilt.disableatpark();},deliveryTilt)
    );
  }
  public double CloseSpeakerShotAddedTiltFromClosestPossibleShot = 0.01;
  public Command C_ReadyCloseSpeakerShot()
  {
    return new InstantCommand(()->{deliveryLifter.setSetpoint(Constants.DeliveryHead.Lift_Position_Zero);},deliveryLifter)
      .alongWith(
        new MoveDTiltToPosition(Constants.DeliveryHead.Tilt_Position_Speaker_Closest+CloseSpeakerShotAddedTiltFromClosestPossibleShot, deliveryTilt),
        //new InstantCommand(()->{deliveryTilt.setSetpointToPosition(Constants.DeliveryHead.Tilt_Position_Speaker_Closest);},deliveryTilt)
        new SpoolPizzaDeliveryToRPM(deliveryShooter, Constants.DeliveryHead.ShooterRpmSpeakerKnownClose)
        );
  }
  // public Command C_ReadyPodiumSpeakerShot()
  // {
  //   return new InstantCommand(()->{deliveryLifter.setSetpoint(Constants.DeliveryHead.Lift_Position_Zero);},deliveryLifter)
  //     .alongWith(
  //       new MoveDTiltToPosition(Constants.DeliveryHead.Tilt_Position_Speaker_Podium, deliveryTilt),
  //       //new InstantCommand(()->{deliveryTilt.setSetpointToPosition(Constants.DeliveryHead.Tilt_Position_Speaker_SafePost);},deliveryTilt),
  //     new SpoolPizzaDeliveryToRPM(deliveryShooter, Constants.DeliveryHead.ShooterRpmSpeakerPodium));
  // }
    public Command C_ReadyCloseAmpShot()
  {
    return new MoveDLifterToPosition(Constants.DeliveryHead.Lift_Position_TrapStart,deliveryLifter)
            .alongWith(
              new MoveDTiltToPosition(Constants.DeliveryHead.Tilt_Position_Amp, deliveryTilt),
              //new InstantCommand(()->{deliveryTilt.setSetpointToPosition(Constants.DeliveryHead.Tilt_Position_Amp);},deliveryTilt),
            new SpoolPizzaDeliveryToRPM(deliveryShooter, Constants.DeliveryHead.ShooterRpmAmp)
            );
    // return new InstantCommand(()->{deliveryLifter.setSetpointPassing();},deliveryLifter)
    //   .alongWith(new InstantCommand(()->{deliveryTilt.setSetpointToPosition(Constants.DeliveryHead.Tilt_Position_Speaker_Closest);},deliveryTilt),
    //new SpoolPizzaDeliveryToRPM(deliveryShooter, Constants.DeliveryHead.ShooterRpmSpeakerClose));
  }
  private ParallelCommandGroup ActivateHotIntake()
  {
   return new ParallelCommandGroup(
     new InstantCommand(()->{deliveryHolder.RequestIndex();}),
      new InstantCommand(()->{PizzaManager.pizzaStage = PizzaTracker.intaking;})
      );
  }
  private ParallelCommandGroup MoveIntoSourcePosition()
  {
     return new ParallelCommandGroup(
      //new MoveDLifterToPosition(Constants.DeliveryHead.Lift_Position_HumanSource,deliveryLifter),
      new MoveDTiltToPosition(Constants.DeliveryHead.Tilt_Position_HumanSourceLow, deliveryTilt),
      new SpoolPizzaDeliveryToRPM(deliveryShooter, Constants.DeliveryHead.ShooterRpmHumanSource),
      new InstantCommand(()->{deliveryHolder.RequestIndex();}),
      new InstantCommand(()->{PizzaManager.pizzaStage = PizzaTracker.intaking;})
      );
  }

  private SequentialCommandGroup HumanSourcePickup() {
    return MoveIntoSourcePosition()
    .andThen(
      new FlashGreenLights(m_candleSubsystem, 20)
      //new WaitCommand(20)
      ).until(isNoteInDeliveryHolderboolSup)
    .andThen(
      C_ParkDeliveryHead().alongWith(new InstantCommand(()->{m_candleSubsystem.StrobeBlueLights();})),
      new WaitCommand(.5),
      new InstantCommand(()->{deliveryHolder.RequestIndex();})
      //new WaitCommand(2)
      //new InstantCommand(()->{m_candleSubsystem.OFFLights();})
      );
  }
  private Command JustShootIt()
  {
    return new SequentialCommandGroup(new WaitForIndexCMD(deliveryHolder),C_ReadyCloseSpeakerShot(),new ShootDeliveryHold(deliveryHolder),new InstantCommand(()->{m_candleSubsystem.StrobeBlueLights();}),  C_ParkDeliveryHead());
  }
    private Command ShootTrapFloor()
  {
    return new SequentialCommandGroup(
      new WaitForIndexCMD(deliveryHolder),
      C_ReadyTrapFloorShot(),
      new ShootDeliveryHold(deliveryHolder),
      new InstantCommand(()->{m_candleSubsystem.StrobeBlueLights();}),  
      C_ParkDeliveryHead());
  }
  public Command C_ReadyTrapFloorShot()
  {
    return new InstantCommand(()->{deliveryLifter.setSetpoint(Constants.DeliveryHead.Lift_Position_Zero);},deliveryLifter)
      .alongWith(
        new TrapShootTILT(deliveryTilt),
        //new InstantCommand(()->{deliveryTilt.setSetpointToPosition(Constants.DeliveryHead.Tilt_Position_Speaker_Closest);},deliveryTilt)
        new TrapShootRPM(deliveryShooter)
        );
  }
  private SequentialCommandGroup PickupRoutine() {
    return new SequentialCommandGroup(
      new MovePickupToPosition(Constants.PizzaFloorPickupHead.PickupFloorPickup, pickuparm).unless(isNoteInIntakeboolSup),
      C_PickupPizzaFromFloorWithoutWashing().unless(isNoteInIntakeboolSup),
      C_ReturnPickupToPassing());
  }
  private Command AlignAndShootCenterSpeaker() {
    return ZeroTilt()
    .andThen(
      new AlignSpeakerCMD(drivetrainManager,() -> joystick.getRawAxis(strafeAxis)).until(IsLimeLightBypassed)
      .alongWith(new InstantCommand(()->{m_candleSubsystem.RainbowRoadLights();},m_candleSubsystem),
      new SequentialCommandGroup(new WaitForIndexCMD(deliveryHolder), C_ReadyCloseSpeakerShot())
      )
      )
    .andThen(//new WaitForIndexCMD(deliveryHolder),
      //C_ReadyCloseSpeakerShot(),
      //new WaitCommand(100),
    (new ShootDeliveryHold(deliveryHolder)),
    new InstantCommand(()->{m_candleSubsystem.StrobeBlueLights();}),
    C_ParkDeliveryHead()
    //new WaitCommand(.5)
    )
    //.onlyIf(()->PizzaManager.NoteInDeliveryHolder)
    ;
  }
  public double AmpBumpTimeout = .45;
  public double SourceBumpTimeout = .6;
  public Command AmpBump()
  {
    return new ForwardBump(drivetrainManager,AmpBumpTimeout);//new SequentialCommandGroup(new ForwardBump(drivetrainManager,.45));   
    
  }
  /////////////////////////////////////////////////////////////

  private void configureBindings() {
    drivetrainManager.configureBindings();
    joystick.back().onTrue(new InstantCommand(()->{PizzaManager.AltControlModeEnabled = !PizzaManager.AltControlModeEnabled;}));
    
    
    joystick.a().onTrue(ActivateHotIntake());//.onFalse(new InstantCommand(()-> {m_candleSubsystem.StrobeWhiteLights();}));strobe should be a command so it ends. 
    //joystick.b().onTrue(new ShootDeliveryHold(deliveryHolder));
    //cancel limelight when not in trap mode 
    joystick.b().and(()->!PizzaManager.AltControlModeEnabled).onTrue(new InstantCommand(()->{SmartDashboard.putBoolean(SDashBoardH.LimelightbypassName, true);})).onFalse(new InstantCommand(()->{SmartDashboard.putBoolean(SDashBoardH.LimelightbypassName, false);}));
    //When in Trap mode we need the option to attemp an overindex again
    joystick.b().and(()->PizzaManager.AltControlModeEnabled).onTrue(
      new InstantCommand(()->{deliveryHolder.m_forwardLimit.enableLimitSwitch(false);
      deliveryHolder.MovePosition(trapindexmovement/2);
      deliveryShooter.MovePosition(shooterIndexMovement/2, true);
      PizzaManager.TrapShotOverIndexed = true;})
            
    );
    joystick.x().and(joystick.rightTrigger().negate()).onTrue(new InstantCommand(()->{deliveryHolder.forceCancelIndex(); pickupSpinner.ReleaseNote();},pickupSpinner).andThen(new RunDeliveryHoldIntake(deliveryHolder,false,40)))
    .onFalse(new InstantCommand(()->{pickupSpinner.stopSpinner();},pickupSpinner));// (pickuparm.runonce(() -> {pickuparm.setSetpointFloorPickup();}));
    joystick.y().and(joystick.rightTrigger().negate()).whileTrue(new InstantCommand(()->{pickupSpinner.RunPickup();}).repeatedly()).onFalse(new InstantCommand(()->{pickupSpinner.stopSpinner();},pickupSpinner)); //.until(()->{pickupSpinner.m_forwardLimit.isPressed();})
    //joystick.y().onTrue(C_PickupPizzaFromFloorWithoutWashing());

    //left trigger will bring joe into the speaker position
    joystick.leftTrigger().whileTrue(AlignWhereverShootSpeaker()//AlignAndShootCenterSpeaker()
    ).onFalse(C_ParkDeliveryHead().andThen(new InstantCommand(()->{deliveryHolder.forceCancelIndex();})));

  //Right Trigger To activate the human pickup
    joystick.rightTrigger().whileTrue(HumanSourcePickup()
    .alongWith(
      new SequentialCommandGroup(
            new AlignSourceCMD(drivetrainManager,joystick).until(IsLimeLightBypassed)
          //new ForwardBump(drivetrainManager,SourceBumpTimeout).unless(IsLimeLightBypassed)
          )
          )
          .andThen(HumanSourcePickup())
      
    )
  .onFalse(C_ParkDeliveryHead());
  
  joystick.rightBumper()
    .onTrue(PickupRoutine())
    .onFalse(C_ReturnPickupHead().unless(isNoteInIntakeboolSup)
    .andThen(new InstantCommand(()->{pickupSpinner.stopSpinner();}),new InstantCommand(()->{m_candleSubsystem.OFFLights();})));//.andThen(C_passNoteFromIntakeToDeliveryHolder().alongWith(C_CatchAndIndexNote().withInterruptBehavior(InterruptionBehavior.kCancelSelf)).onlyIf(isNoteInIntakeboolSup)).andThen(new InstantCommand(()->{pickupSpinner.stopSpinner();})));

  /////PODIUM SHOT TEST
    // joystick.rightTrigger().whileTrue(
    //   new InstantCommand(()->{deliveryLifter.setSetpointZero();},deliveryLifter)
    //   .alongWith(new InstantCommand(()->{deliveryTilt.setSetpointToPosition(Constants.DeliveryHead.Tilt_Position_Speaker_SafePost);},deliveryTilt),new InstantCommand(()->deliveryShooter.SetShootSpeed(Constants.DeliveryHead.ShooterRpmSpeakerClose))).onlyIf(()->PizzaManager.NoteInDeliveryHolder))
    //   .onFalse(C_ParkDeliveryHead());
  /////


    //OLD left bumper will bring joe into the amp position 
    joystick.leftBumper()
    .whileTrue(
        new ParallelCommandGroup(
        new AlignAmpCMD(drivetrainManager,() -> joystick.getRawAxis(strafeAxis)),
        new MoveDLifterToPosition(Constants.DeliveryHead.Lift_Position_Amp,deliveryLifter),
        new InstantCommand(()->{m_candleSubsystem.RainbowRoadLights();},m_candleSubsystem)  
        //new ForwardBump(drivetrainManager,AmpBumpTimeout).unless(IsLimeLightBypassed)
        ).until(IsLimeLightBypassed)
        .andThen(
              new ParallelCommandGroup(
              new MoveDLifterToPosition(Constants.DeliveryHead.Lift_Position_Amp,deliveryLifter), 
              new MoveDTiltToPosition(Constants.DeliveryHead.Tilt_Position_Amp,deliveryTilt,Constants.DeliveryHead.Tilt_Amp_Tolerance,Constants.DeliveryHead.TiltSettleTimeAtPosition/3),
              new SpoolPizzaDeliveryToRPM(deliveryShooter, Constants.DeliveryHead.ShooterRpmAmp,25)
              )
              .andThen( 
                (new ShootDeliveryHold(deliveryHolder,.5,.5)),//DUMP MIN TIMEOUT ! ->this is the dump time so we dont flick a note back
                //new WaitCommand(.05),//old dump timeout
                new InstantCommand(()->{m_candleSubsystem.StrobeBlueLights();},m_candleSubsystem),
              C_ParkDeliveryHead()
              //.onlyIf(()->PizzaManager.NoteInDeliveryHolder)
              )
            )
    )
    .onFalse(C_ParkDeliveryHead());
    /////
    //OLD left bumper will bring joe into the amp position 
    // joystick.leftBumper().whileTrue(
    //   new InstantCommand(()->{deliveryLifter.setSetpointAmp();},deliveryLifter)
    //         .alongWith(new InstantCommand(()->{deliveryTilt.setSetpointToPosition(Constants.DeliveryHead.Tilt_Position_Amp);},deliveryTilt),
    //         new InstantCommand(()->deliveryShooter.SetShootSpeed(Constants.DeliveryHead.ShooterRpmAmp))
    //         ).onlyIf(()->PizzaManager.NoteInDeliveryHolder)
    //         ).onFalse(C_ParkDeliveryHead());
    /////
    //Align Trap Shoot Test
    joystick.pov(Constants.XboxControllerMap.kPOVDirectionRIGHT).and(()->!PizzaManager.AltControlModeEnabled)
    .whileTrue(
      new AlignTrapShootCMD(drivetrainManager,() -> joystick.getRawAxis(strafeAxis)).andThen(ShootTrapFloor(),C_ParkDeliveryHead())
    )
    .onFalse(C_ParkDeliveryHead());
    //
    ///////////////////////
  joystick.pov(Constants.XboxControllerMap.kPOVDirectionLeft).and(()->!PizzaManager.AltControlModeEnabled)
    .whileTrue(AlignAndShootCenterSpeaker()//AlignWhereverShootSpeaker()
    )
    .onFalse(C_ParkDeliveryHead());
//////////
joystick.pov(Constants.XboxControllerMap.kPOVDirectionDOWN).and(()->!PizzaManager.AltControlModeEnabled)
.onTrue(new ZeroDtilt(deliveryTilt).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
/////////////
    ///////////
    //OLD ALIGNMENT TEST AMP
    // joystick.pov(Constants.XboxControllerMap.kPOVDirectionLeft).and(()->!PizzaManager.AltControlModeEnabled)
    //     .whileTrue(new AlignAmpCMD2(drivetrainManager,() -> joystick.getRawAxis(strafeAxis)).unless(IsLimeLightBypassed)
    // .andThen(
    //   C_ReadyCloseAmpShot(),
    //   //new WaitCommand(1),
    // (new ShootDeliveryHold(deliveryHolder)),
    // C_ParkDeliveryHead()))
    // .onFalse(C_ParkDeliveryHead());
    //
    // .whileTrue(new AlignSpeakerCMD(drivetrainManager,LL3,() -> joystick.getRawAxis(strafeAxis))
    // .andThen(
    //   C_ReadyCloseSpeakerShot(),
    //   //new WaitCommand(1),
    // (new ShootDeliveryHold(deliveryHolder)),
    // C_ParkDeliveryHead()))
    // .onFalse(C_ParkDeliveryHead());

    ///
    // joystick.pov(Constants.XboxControllerMap.kPOVDirectionRIGHT).and(()->!PizzaManager.AltControlModeEnabled)
    // .whileTrue(C_ReadyCloseSpeakerShot())
    // .onFalse(C_ParkDeliveryHead());
    //
    //
    //joystick.pov(Constants.XboxControllerMap.kPOVDirectionRIGHT).and(()->!PizzaManager.AltControlModeEnabled)
    //.whileTrue(drivetrainManager.drivetrain.getAutoPath("amp bump"));
    //  
    // //
    // joystick.pov(Constants.XboxControllerMap.kPOVDirectionRIGHT).and(()->!PizzaManager.AltControlModeEnabled)
    // .whileTrue(new InstantCommand(()->{m_candleSubsystem.RainbowRoadLights();})).onFalse(new InstantCommand(()->{m_candleSubsystem.GreenLights();}));
    // //  
    //  joystick.pov(Constants.XboxControllerMap.kPOVDirectionRIGHT).and(()->!PizzaManager.AltControlModeEnabled)
      //  .whileTrue(
      // new InstantCommand(()->{deliveryLifter.setSetpoint(Constants.DeliveryHead.Lift_Position_Trap);},deliveryLifter)
      // .alongWith(new InstantCommand(()->{deliveryTilt.setSetpointToPosition(Constants.DeliveryHead.Tilt_Position_Amp);},deliveryTilt)))
      //  .onFalse(new InstantCommand(()->{deliveryLifter.setSetpointZero();},deliveryLifter).alongWith(C_TiltGotoPark()));
      //.whileTrue(new AlignSpeakerTest(drivetrainManager,LL3));
    // ////////////
    // joystick.pov(Constants.XboxControllerMap.kPOVDirectionUP).and(()->!PizzaManager.AltControlModeEnabled).whileTrue(new InstantCommand(()->{ChainLift.AlterSetpointposition(10);}));
    // joystick.pov(Constants.XboxControllerMap.kPOVDirectionDOWN).and(()->!PizzaManager.AltControlModeEnabled).whileTrue(new InstantCommand(()->{ChainLift.AlterSetpointposition(-10);}));
    // ////////
    //////////////////
    //joystick.pov(Constants.XboxControllerMap.kPOVDirectionUP).and(()->!PizzaManager.AltControlModeEnabled)
    //.whileTrue(drivetrainManager.runAuto);
    //////////////
    // joystick.pov(Constants.XboxControllerMap.kPOVDirectionUP).and(()->!PizzaManager.AltControlModeEnabled)
    // .whileTrue(C_CatchAndIndexNote()
    // .andThen(
    //   AlignAndShootCenterSpeaker()//,
    //   //PickupRoutine().alongWith(getAutonomousCommandSingle()),
    //   //AutoBuilder.followPath(PathPlannerPath.fromPathFile("GotoCenterFromCenter")),
    //   //AlignAndShootCenterSpeaker(),
    //   //PickupRoutine().alongWith(AutoBuilder.followPath(PathPlannerPath.fromPathFile("backupToMid"))),
    //   //AutoBuilder.followPath(PathPlannerPath.fromPathFile("GotoCenterFromCenter"))
    //   )
    //   );
//////////////////
    // joystick.pov(Constants.XboxControllerMap.kPOVDirectionDOWN).and(()->!PizzaManager.AltControlModeEnabled)
    // .whileTrue(drivetrainManager.drivetrain.applyRequest(() -> drivetrainManager.drive.withVelocityX(-.1 * drivetrainManager.MaxSpeed) // Drive forward with                                                                                        // negative Y (forward)
    //         .withVelocityY(-.1 * drivetrainManager.MaxSpeed) // Drive left with negative X (left)
    //         .withRotationalRate(0 * drivetrainManager.MaxAngularRate) // Drive counterclockwise with negative X (left)
    //     ).ignoringDisable(false)).onFalse(
    //       drivetrainManager.drivetrain.applyRequest(() -> drivetrainManager.drive.withVelocityX(0 * drivetrainManager.MaxSpeed) // Drive forward with                                                                                        // negative Y (forward)
    //         .withVelocityY(0 * drivetrainManager.MaxSpeed) // Drive left with negative X (left)
    //         .withRotationalRate(0 * drivetrainManager.MaxAngularRate) // Drive counterclockwise with negative X (left)
    //     ).ignoringDisable(false)
    //     );
      //////////////
     //joystick.pov(Constants.XboxControllerMap.kPOVDirectionRIGHT).onTrue(new InstantCommand(() -> {deliveryTilt.AlterSetpointposition(-.2);}));
    //if in Amp positions or Shoot position, pressing A will execute that action. 
    //joystick.a().whileTrue(runAuto);

    //joystick.rightTrigger().onTrue(new InstantCommand(()->{deliveryTilt.setSetpointZero();},deliveryTilt));
    
    //.onTrue(new InstantCommand(()->{deliveryTilt.setSetpointPassing();},deliveryTilt));
    configureTrapButtons();
  }
  public Command ZeroTilt()
  {
    return new InstantCommand(()->{});//ZeroDtilt(deliveryTilt).unless(()->PizzaManager.HasTiltBeenZeroed).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
  }
public Command AlignWhereverShootSpeaker()
{
  return ZeroTilt()
      .andThen(
        new AlignSpeakerTest(drivetrainManager,deliveryTilt,deliveryShooter, m_candleSubsystem,() -> joystick.getRawAxis(0), () -> joystick.getRawAxis(1)),
        new WaitForIndexCMD(deliveryHolder),
        //C_ReadyCloseSpeakerShot(),
        new ShootDeliveryHold(deliveryHolder),
        //new WaitCommand(10),
        C_ParkDeliveryHead()
        );
}

public double trapindexmovement = 60;//80;
public double shooterIndexMovement = 1.15;//1.55;

  public void configureTrapButtons()
  {
    //going DOWN
    joystick.pov(Constants.XboxControllerMap.kPOVDirectionDOWN).and(()->PizzaManager.AltControlModeEnabled)
    .onTrue(new MoveChainLiftToPosition(Constants.ChainLifter.Lift_Position_Zero, ChainLift)//.alongWith(new InstantCommand(()->{deliveryTilt.setSetpointToPosition(Constants.DeliveryHead.Tilt_Position_Park);}))
    .andThen(
      new MovePickupToPosition(Constants.PizzaFloorPickupHead.PickupPassing, pickuparm)
    .alongWith(
      C_ParkDeliveryHead(),
      new InstantCommand(()->{deliveryHolder.m_forwardLimit.enableLimitSwitch(true);}),
      new InstantCommand(()->{ChainLift.disableatpark();},ChainLift)
      )
    )
    .andThen(
      //new InstantCommand(()->{PizzaManager.TrapShotOverIndexed = false;}),
      new InstantCommand(()->{deliveryHolder.RequestIndex(PizzaTracker.intaking);}),
      new SpoolPizzaDeliveryToRPM(deliveryShooter, Constants.DeliveryHead.ShooterRpmHumanSource/2)
      
      )
    );
    
    //going DOWN
    // joystick.pov(Constants.XboxControllerMap.kPOVDirectionDOWN).and(()->PizzaManager.AltControlModeEnabled)
    // .onTrue(new MoveChainLiftToPosition(Constants.ChainLifter.Lift_Position_Zero, ChainLift)//.alongWith(new InstantCommand(()->{deliveryTilt.setSetpointToPosition(Constants.DeliveryHead.Tilt_Position_Park);}))
    // .andThen(new MovePickupToPosition(Constants.PickupHead.PickupPassing, pickuparm)
    // .alongWith(C_ParkDeliveryHead(),new InstantCommand(()->{deliveryHolder.m_forwardLimit.enableLimitSwitch(true);}))
    // ));
    
    joystick.pov(Constants.XboxControllerMap.kPOVDirectionRIGHT).and(()->PizzaManager.AltControlModeEnabled)
    .onTrue(
      new InstantCommand(() -> {})//deliveryTilt.setSetpointToPosition(Constants.DeliveryHead.Tilt_Position_TrapDodge);
      .andThen(
        new MoveChainLiftToPosition(Constants.ChainLifter.Lift_Position_PullDown, ChainLift)
        
      .alongWith(
          new SequentialCommandGroup(
              new WaitCommand(.75), 
              //new InstantCommand(()->{deliveryHolder.MovePosition(-trapindexmovement);}),
              ActivateHotIntake(),
              new InstantCommand(()->{deliveryTilt.setSetpointToPosition(Constants.DeliveryHead.Tilt_Position_TrapLiftUp);})
              )
          
      .andThen(new WaitCommand(.25),
      new InstantCommand(()->{deliveryHolder.RequestIndex(PizzaTracker.intaking); }), //PizzaManager.TrapShotOverIndexed = false;
      new SpoolPizzaDeliveryToRPM(deliveryShooter, Constants.DeliveryHead.ShooterRpmHumanSource/2)
       
      )
      .andThen(new MovePickupToPosition(Constants.PizzaFloorPickupHead.PickupFloorPickup, pickuparm))
      .andThen(new InstantCommand(()->{deliveryLifter.setSetpoint(Constants.DeliveryHead.Lift_Position_Trap);},deliveryLifter))
      .andThen(new WaitCommand(2.0))
      .andThen(new MovePickupToPosition(Constants.PizzaFloorPickupHead.PickupVertical, pickuparm))
      )
      )
      );


    //alignment is not in the alt controlled mode!
     joystick.pov(Constants.XboxControllerMap.kPOVDirectionUP).and(()->!PizzaManager.AltControlModeEnabled)
    .whileTrue(
      new AlignStageCMD(drivetrainManager,() -> joystick.getRawAxis(strafeAxis)).until(IsLimeLightBypassed)
      .alongWith(new InstantCommand(()->{LimelightHelpers.setLEDMode_ForceOn(Constants.LimelightName);}))
    );
    //unfold and get ready for lift
    joystick.pov(Constants.XboxControllerMap.kPOVDirectionUP).and(()->PizzaManager.AltControlModeEnabled)
    .onTrue(
      new MovePickupToPosition(Constants.PizzaFloorPickupHead.PickupVertical, pickuparm)
    .alongWith(new InstantCommand(()->{
      LimelightHelpers.setLEDMode_ForceOff(Constants.LimelightName);
      deliveryHolder.m_forwardLimit.enableLimitSwitch(false);
      deliveryHolder.MovePosition(trapindexmovement);
      deliveryShooter.MovePosition(shooterIndexMovement, true);
      PizzaManager.TrapShotOverIndexed = true;
    },deliveryHolder).unless(()->PizzaManager.TrapShotOverIndexed),
      
      new InstantCommand(()->{deliveryLifter.setSetpoint(Constants.DeliveryHead.Lift_Position_TrapStart);},deliveryLifter))
    .alongWith(new InstantCommand(()->{deliveryTilt.setSetpointToPosition(Constants.DeliveryHead.Tilt_Position_TrapLift);}))
    .andThen(
      new MoveChainLiftToPosition(Constants.ChainLifter.Lift_Position_ForDeliveryKick, ChainLift),
      new InstantCommand(() -> {deliveryTilt.setSetpointToPosition(Constants.DeliveryHead.Tilt_Position_TrapLiftHitHook);}),
      new MoveChainLiftToPosition(Constants.ChainLifter.Lift_Position_CenterAndTrap, ChainLift))
      );
    
    joystick.pov(Constants.XboxControllerMap.kPOVDirectionLeft).and(()->PizzaManager.AltControlModeEnabled)
    .onTrue(new MoveDTiltToPosition(Constants.DeliveryHead.Tilt_Position_TrapLiftUpSHOOT, deliveryTilt,Constants.DeliveryHead.Tilt_Amp_Tolerance,Constants.DeliveryHead.TiltSettleTimeAtPosition/3)
    .alongWith(
      new MoveDLifterToPosition(Constants.DeliveryHead.Lift_Position_TrapShoot, deliveryLifter),
      new MovePickupToPosition(Constants.PizzaFloorPickupHead.PickupFloorPickup, pickuparm)
    )
    .andThen(
      new InstantCommand(()->deliveryShooter.SetShootSpeed(Constants.DeliveryHead.ShooterRpmSpeakerClose)),
      new WaitCommand(.2),
      new ShootDeliveryHold(deliveryHolder),
      new InstantCommand(()->{ChainLift.disableatpark();})
      )//new ShootDeliveryHold(deliveryHolder))
    
    )
    .onFalse(new InstantCommand(()->deliveryShooter.SetShootSpeed(Constants.DeliveryHead.ShooterRpmOff)).andThen(new InstantCommand(()->{deliveryHolder.requestingIndex = false; deliveryHolder.SetToWantedDutyCycle(0);}))
    .alongWith(
      new InstantCommand(() -> {deliveryTilt.setSetpointToPosition(Constants.DeliveryHead.Tilt_Position_TrapLiftUp);})
      .andThen(new MovePickupToPosition(Constants.PizzaFloorPickupHead.PickupVertical, pickuparm))));
    //////
  }
  public RobotContainer() {
    
    BuildAutos();
    //
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto choices", autoChooser);
    SmartDashboard.setPersistent("Auto choices");
    //
    PizzaManager.IsOvenPickUpRunning = joystick.rightTrigger(.1);

    configureBindings();
    m_candleSubsystem.StartupLights();
  }

  private void BuildAutos() {
          // Register Named Commands
      NamedCommands.registerCommand("marker1", Commands.print("Passed marker 1"));
      NamedCommands.registerCommand("marker2", Commands.print("Passed marker 2"));
      NamedCommands.registerCommand("print hello", Commands.print("hello"));
      NamedCommands.registerCommand("AlignAndShootCenterSpeaker", AlignAndShootCenterSpeaker());
      NamedCommands.registerCommand("C_CatchAndIndexNote", C_CatchAndIndexNote());
     // NamedCommands.registerCommand("WaitForIndex", new WaitForIndexCMD(deliveryHolder));
      NamedCommands.registerCommand("PickupRoutine", PickupRoutine());
      NamedCommands.registerCommand("ReadyShootPreEmptive", ReadyShootPreEmptive());
      NamedCommands.registerCommand("JustShootIt",   JustShootIt());
      NamedCommands.registerCommand("AlignWhereverShootSpeaker",   AlignWhereverShootSpeaker());
      NamedCommands.registerCommand("Placeholder",   PlaceHolder());
      NamedCommands.registerCommand("ZeroDTilt", PlaceHolder());//ZeroDtilt()
    
      
      //Now build the autos 
      drivetrainManager.runAuto = drivetrainManager.drivetrain.getAutoPath("SP_C_2Piece");
  }
  private Command ZeroDtilt()
  {
    return new ZeroDtilt(deliveryTilt);
  }
  private Command PlaceHolder() {
    return new InstantCommand(()->{});
  }
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
    /* First put the drivetrain into auto run mode, then run the auto */
    //return drivetrainManager.runAuto;
  }
  public Command getAutonomousCommandSingle() {
        // Load the path you want to follow using its name in the GUI
        PathPlannerPath path = PathPlannerPath.fromPathFile("backupCenter");

        // Create a path following command using AutoBuilder. This will also trigger event markers.
        return AutoBuilder.followPath(path);
    }
  

}
