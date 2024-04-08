package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DeliveryTilt extends SubsystemBase {

    
    
    
    public String MotorName = "DeliveryTilt";
    public double CurrentEncoderValue = 0;
    public double WantedEncoderValue = 0;
    public double CurrentEncoderVelocity = 0;
    public double OutputCurrent = 0;
    public double MotorTemp = 0;
    public double TempCForOverTemp = 37;
    double kMaxOutput = 1; 
    double kMinOutput = -1;



      public double CruiseVelocity = 0.001;
      public double Acceleration = 0.001;
      public double Jerk = 0.005;
  

    double kFF = 0.00;
    double kIz = 0;

    public final CANcoder TiltEncoder = new CANcoder(Constants.CANBus.Tilt_ABSOCANCODER_CanBusID,Constants.CANBus.kRIOCANbusName);
    public final TalonFX Motor_Controller = new TalonFX(Constants.CANBus.Tilt_CanBusID,Constants.CANBus.kRIOCANbusName);
      /* Start at position 0, no feed forward, use slot 1 */
    private final PositionTorqueCurrentFOC m_torquePosition = new PositionTorqueCurrentFOC(0, 0, 0, 1, false, false, false);
    
    //private final MotionMagicTorqueCurrentFOC m_MMtorquePosition = new MotionMagicTorqueCurrentFOC(0, 0, 1, false, false, false);

    /* Keep a neutral out so we can disable the motor */
    private final NeutralOut m_brake = new NeutralOut();
    //private final CANSparkMax Motor_Controller = new CANSparkMax(Constants.CANBus.Tilt_CanBusID, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
    //private final RelativeEncoder Motor_Encoder = Motor_Controller.getEncoder();
    //private final SparkPIDController MotorControllerPid = Motor_Controller.getPIDController();
    
  private final StatusSignal<Double> cc_pos = TiltEncoder.getPosition();

    private int ZeroAttempts = 0;
    public DeliveryTilt()
    {
        SetupMotorConfig();
        
        //should the motor controller be inverted? 0 is folded in and 44 (or max) is folded out.
        Motor_Controller.setInverted(false);
        Motor_Controller.setNeutralMode(NeutralModeValue.Brake);
        // display PID coefficients on SmartDashboard
        SmartDashboard.putNumber(MotorName + " P Gain", Constants.DeliveryHead.kP_Tilter);
        SmartDashboard.putNumber(MotorName + " I Gain", Constants.DeliveryHead.kI_Tilter);
        SmartDashboard.putNumber(MotorName + " D Gain", Constants.DeliveryHead.kD_Tilter);
        //SmartDashboard.putNumber(MotorName + " I Zone", kIz);
        //SmartDashboard.putNumber(MotorName + " Feed Forward", kFF);

        //SmartDashboard.putNumber(MotorName + " MMAcceleration", Acceleration);
       // SmartDashboard.putNumber(MotorName + " MMCruiseVelocity", CruiseVelocity);
       // SmartDashboard.putNumber(MotorName + " MMJerk", Jerk);

        SmartDashboard.putNumber(MotorName + " Closest Setpoint", Constants.DeliveryHead.Tilt_Position_Speaker_Closest);
        SmartDashboard.putNumber(MotorName + " Mid Setpoint", Constants.DeliveryHead.Tilt_Position_Speaker_Mid);
        SmartDashboard.putNumber(MotorName + " Furthest Setpoint", Constants.DeliveryHead.Tilt_Position_Speaker_Furthest);

        StatusCode status = Motor_Controller.setPosition(0);
        ZeroAttempts++;
        if(!status.isOK()) {
          for(int i =0; i<10; i++) {
            ZeroAttempts++;
            System.out.println("Could not apply zero position, error code: " + status.toString());
            status = Motor_Controller.setPosition(0,250);
            if(status.isOK()) break;
          }
          SmartDashboard.putBoolean(MotorName + "SetPosition 0", status.isOK());
          
        }
        
        
    }
    double Absoluteposition = 0.0;
    public void getEncoderData()
    {
      //fx_pos.refresh(); 
      //fx_vel.refresh();
      cc_pos.refresh(); 
      //cc_vel.refresh();
      //System.out.println("FX Position: " + fx_pos + " FX Vel: " + fx_vel);
      //System.out.println("CC Position: " + cc_pos + " CC Vel: " + cc_vel);

      Absoluteposition = TiltEncoder.getAbsolutePosition().getValueAsDouble();
      SmartDashboard.putNumber(MotorName + " AbsoPos",cc_pos.getValueAsDouble());


      SmartDashboard.putNumber(MotorName + " ZeroAttempts",ZeroAttempts);

      OutputCurrent = Motor_Controller.getTorqueCurrent().getValueAsDouble();;
      SmartDashboard.putNumber(MotorName + " Amps",OutputCurrent);
  
      MotorTemp = Motor_Controller.getDeviceTemp().getValueAsDouble();
      SmartDashboard.putNumber(MotorName + " Motor Temp",MotorTemp);
      /**
       * Encoder position is read from a RelativeEncoder object by calling the
       * GetPosition() method.
       * 
       * GetPosition() returns the position of the encoder in units of revolutions
       */
      CurrentEncoderValue = Motor_Controller.getPosition().getValueAsDouble();
      SmartDashboard.putNumber(MotorName + " PID Encoder Position",CurrentEncoderValue);
  
      /**
       * Encoder velocity is read from a RelativeEncoder object by calling the
       * GetVelocity() method.
       * 
       * GetVelocity() returns the velocity of the encoder in units of RPM
       */
      CurrentEncoderVelocity = Motor_Controller.getVelocity().getValueAsDouble();
      //SmartDashboard.putNumber(MotorName + " RPM", CurrentEncoderVelocity*60);
      
      SmartDashboard.putNumber(MotorName + " PID output",Motor_Controller.getClosedLoopOutput().getValueAsDouble());
      SmartDashboard.putNumber(MotorName + " setpoint ",  WantedEncoderValue);
      

    }

    @Override
    public void periodic() {
        //anything you wanted to do periodically put it here.
      //INSIDE GET ENCODER DATA WE UPDATE OUR CurrentLiftEncoderValue! this is how the PID WORKS!!!!
      getEncoderData();
      //super.periodic();// This is a PidSubsystem, we have orridden the periodic method to get encoder data... So we need to call the super periodic method to get the PID stuff to work.
      // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber(MotorName + " P Gain", 0);
    double i = SmartDashboard.getNumber(MotorName + " I Gain", 0);
    double d = SmartDashboard.getNumber(MotorName + " D Gain", 0);
    double mintilt =  SmartDashboard.getNumber(MotorName + " Closest Setpoint", Constants.DeliveryHead.Tilt_Position_Speaker_Closest);
    double maxtilt =  SmartDashboard.getNumber(MotorName + " Furthest Setpoint", Constants.DeliveryHead.Tilt_Position_Speaker_Furthest);
    double midtilt =  SmartDashboard.getNumber(MotorName + " Mid Setpoint", Constants.DeliveryHead.Tilt_Position_Speaker_Mid);

   // double Accel2 = SmartDashboard.getNumber(MotorName + " MMAcceleration", Acceleration);
   // double Cruise2 = SmartDashboard.getNumber(MotorName + " MMCruiseVelocity", CruiseVelocity);
   // double  Jerk2 = SmartDashboard.getNumber(MotorName + " MMJerk", Jerk);



    if((Constants.DeliveryHead.Tilt_Position_Speaker_Closest != mintilt)) { Constants.DeliveryHead.Tilt_Position_Speaker_Closest = mintilt; }
    if((Constants.DeliveryHead.Tilt_Position_Speaker_Mid != midtilt)) { Constants.DeliveryHead.Tilt_Position_Speaker_Mid = midtilt;}
    if((Constants.DeliveryHead.Tilt_Position_Speaker_Furthest != maxtilt)) { Constants.DeliveryHead.Tilt_Position_Speaker_Furthest = maxtilt;}//System.out.println("MaxTiltSet");

    if((p != Constants.DeliveryHead.kP_Tilter)) { configs.Slot1.kP = p; Constants.DeliveryHead.kP_Tilter = p; SetConfigToMotor(); }
     if((i != Constants.DeliveryHead.kI_Tilter)) { configs.Slot1.kI = i; Constants.DeliveryHead.kI_Tilter = i; SetConfigToMotor(); }
     if((d != Constants.DeliveryHead.kD_Tilter)) { configs.Slot1.kD = d; Constants.DeliveryHead.kD_Tilter = d; SetConfigToMotor(); }
     //if((Accel2 != Acceleration)) { configs.MotionMagic.MotionMagicAcceleration = Accel2; Acceleration = Accel2; SetConfigToMotor(); }
     // if((Cruise2 != CruiseVelocity)) { configs.MotionMagic.MotionMagicCruiseVelocity = Cruise2; CruiseVelocity = Cruise2; SetConfigToMotor(); }
     // if((Jerk2 != Jerk)) { configs.MotionMagic.MotionMagicJerk = Jerk2; Jerk = Jerk2; SetConfigToMotor(); }

    }

    public void SetSpeed(double thisspeed) {
        Motor_Controller.set(thisspeed);
      }
    
      TalonFXConfiguration configs = new TalonFXConfiguration();

      private void SetupMotorConfig() {
      

          /* Configure CANcoder to zero the magnet appropriately */

      CANcoderConfiguration cc_cfg = new CANcoderConfiguration();
      cc_cfg.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
      cc_cfg.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
      cc_cfg.MagnetSensor.MagnetOffset = Constants.DeliveryHead.AbsoluteEncoderOffset;
      TiltEncoder.getConfigurator().apply(cc_cfg);
      ///////////
      //////////////
       var AbsoluteEncoderFeedbackConfig = new FeedbackConfigs().withFeedbackRemoteSensorID(TiltEncoder.getDeviceID())
       .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
       .withRotorToSensorRatio(Constants.DeliveryHead.TiltGearRatio).withSensorToMechanismRatio(1.0);
      
      //configs.Feedback.FeedbackRemoteSensorID = TiltEncoder.getDeviceID();
      //configs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
      //configs.Feedback.SensorToMechanismRatio = 1.0;
      //configs.Feedback.RotorToSensorRatio = 80.0;
      //
      //
      /* Configure current limits */
      //MotionMagicConfigs mm = configs.MotionMagic;
      //mm.MotionMagicCruiseVelocity = CruiseVelocity; // 5 rotations per second cruise
      //mm.MotionMagicAcceleration = Acceleration; // Take approximately 0.5 seconds to reach max vel
      // Take approximately 0.2 seconds to reach max accel 
      //mm.MotionMagicJerk = Jerk;
      //
      configs.Slot1.kP = Constants.DeliveryHead.kP_Tilter;//40; // An error of 1 rotations results in 40 amps output
      configs.Slot1.kI = Constants.DeliveryHead.kI_Tilter;//0;
      configs.Slot1.kD = Constants.DeliveryHead.kD_Tilter;//2; // A change of 1 rotation per second results in 2 amps output
      // Peak output of 130 amps
      configs.TorqueCurrent.PeakForwardTorqueCurrent = 40;
      configs.TorqueCurrent.PeakReverseTorqueCurrent = -40;
      
      configs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
      configs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

      configs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.DeliveryHead.Tilt_maxValue;
      configs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.DeliveryHead.Tilt_minValue-5;
      
      configs.withFeedback(AbsoluteEncoderFeedbackConfig);//.withFeedbackRotorOffset(0)
      //configs.withFeedback(new FeedbackConfigs().withFeedbackRotorOffset(0));//commented as a test is this interfering with the other feedback config. 

      SetConfigToMotor();


    }

      private void SetConfigToMotor() {
        /* Retry config apply up to 5 times, report if failure */
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
          //PUT MOTORS TO BE CONFIGED HERE
          status = Motor_Controller.getConfigurator().apply(configs);
          //
          if (status.isOK()) break;
        }
        if(!status.isOK()) {
          System.out.println("Could not apply configs, error code: " + status.toString());
        }
      }

      public void AlterSetpointposition(double AddToPosition)
      {
        WantedEncoderValue = WantedEncoderValue + AddToPosition;
        Motor_Controller.setControl(m_torquePosition.withPosition(WantedEncoderValue));
        //MotorControllerPid.setReference(WantedEncoderValue, CANSparkBase.ControlType.kPosition);
      }

      public void setSetpointToPosition(double position)
      {
        //enable();
        WantedEncoderValue = position;
        //Motor_Controller.setControl(m_MMtorquePosition.withPosition(WantedEncoderValue));
        Motor_Controller.setControl(m_torquePosition.withPosition(WantedEncoderValue));
        //MotorControllerPid.setReference(WantedEncoderValue, CANSparkBase.ControlType.kPosition);
      }
      // public void setsHOTSetpointToPosition(double position)
      // {
      //   //enable();
      //   WantedEncoderValue = position;
      //   Motor_Controller.setControl(m_MMtorquePosition.withPosition(WantedEncoderValue));
      //   //Motor_Controller.setControl(m_torquePosition.withPosition(WantedEncoderValue));
      //   //MotorControllerPid.setReference(WantedEncoderValue, CANSparkBase.ControlType.kPosition);
      // }

      public void setSetpointZero() {
        //enable();
        WantedEncoderValue = Constants.DeliveryHead.Tilt_Position_Zero;
        //Motor_Controller.setControl(m_MMtorquePosition.withPosition(WantedEncoderValue));
        Motor_Controller.setControl(m_torquePosition.withPosition(WantedEncoderValue));
      }
        public void setSetpointPassing() {
        //enable();
        WantedEncoderValue = Constants.DeliveryHead.Tilt_Position_Passing;
        //Motor_Controller.setControl(m_MMtorquePosition.withPosition(WantedEncoderValue));
        Motor_Controller.setControl(m_torquePosition.withPosition(WantedEncoderValue));
      }
      public void setSetpointAmp() {
        //enable();
        WantedEncoderValue = Constants.DeliveryHead.Tilt_Position_Amp;
        //Motor_Controller.setControl(m_MMtorquePosition.withPosition(WantedEncoderValue));
        Motor_Controller.setControl(m_torquePosition.withPosition(WantedEncoderValue));
      }
      public void setSetpointTrapFloor() 
      {
        //enable();
        WantedEncoderValue = Constants.DeliveryHead.Tilt_Position_TrapFloorShoot;
        //Motor_Controller.setControl(m_MMtorquePosition.withPosition(WantedEncoderValue));
        Motor_Controller.setControl(m_torquePosition.withPosition(WantedEncoderValue));
      }


      //public boolean HasNote = false;
      public void disableatpark()
      {
        setSetpointToPosition(0);//CurrentEncoderValue
        //Motor_Controller.stopMotor();
        Motor_Controller.setControl(m_brake);//we press into our hysterisis on powerup. so without this the tilt motor always runs trying to go to the bottom. 
      }
      public void resetSettleTimer()
      {
        m_SettleTimer.reset();
        m_SettleTimer.start();
      }

      //double SettleTimeAtCorrectTilt = 0.1;//0.2;

      private final Timer m_SettleTimer = new Timer();
      // public boolean atSetpoint() {    
          
      //     //auton version
      //     if(DriverStation.isAutonomousEnabled())
      //     {
      //       if (Constants.isWithinAmount(CurrentEncoderValue, WantedEncoderValue, Constants.DeliveryHead.TiltsetpointTolerance*autonToleranceMultipler)) 
      //       {
      //         if(m_SettleTimer.get() > AutonSettleTimeAtCorrectTilt)
      //         {
      //         return true;
      //         }
      //       }
      //       else {
      //       resetSettleTimer();
      //       }
      //       return false;   
      //     }
      //     ////teleop version
      //     if (Constants.isWithinAmount(CurrentEncoderValue, WantedEncoderValue, Constants.DeliveryHead.TiltsetpointTolerance)) {
      //       if(m_SettleTimer.get() > SettleTimeAtCorrectTilt){
      //         return true;
      //       }
      //     } else {
      //       resetSettleTimer();
      //     }
      //     return false; 
      // }
      public boolean atSetpoint(double TiltTolerance, double SettleTime) {    

          if (Constants.isWithinAmount(CurrentEncoderValue, WantedEncoderValue, TiltTolerance)) {
            if(m_SettleTimer.get() > SettleTime){
              SmartDashboard.putBoolean(MotorName + "InTarget ",  true);
              return true;
            }
          } else {
            resetSettleTimer();
          }
          SmartDashboard.putBoolean(MotorName + "InTarget ",  false);
          return false; 
      }
      public boolean isMotorOvertemp()
      {
        if(MotorTemp >TempCForOverTemp)
        {
          return true;
        }
        else
        {
          return false;
        }
      }
}
