package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

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

    double kP_Tilter = 0.0700;
    double kI_Tilter = 0.000004;
    double kD_Tilter = 0.000001;

    double kFF = 0.00;
    double kIz = 0;

    private final TalonFX Motor_Controller = new TalonFX(Constants.CANBus.Tilt_CanBusID,Constants.CANBus.kRIOCANbusName);
      /* Start at position 0, no feed forward, use slot 1 */
    private final PositionTorqueCurrentFOC m_torquePosition = new PositionTorqueCurrentFOC(0, 0, 0, 1, false, false, false);
    /* Keep a neutral out so we can disable the motor */
    private final NeutralOut m_brake = new NeutralOut();
    //private final CANSparkMax Motor_Controller = new CANSparkMax(Constants.CANBus.Tilt_CanBusID, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
    //private final RelativeEncoder Motor_Encoder = Motor_Controller.getEncoder();
    //private final SparkPIDController MotorControllerPid = Motor_Controller.getPIDController();
    
 
    public DeliveryTilt()
    {
        SetupMotorConfig();
        Motor_Controller.setPosition(0);
        //should the motor controller be inverted? 0 is folded in and 44 (or max) is folded out.
        Motor_Controller.setInverted(false);
        
        // display PID coefficients on SmartDashboard
        SmartDashboard.putNumber(MotorName + " P Gain", kP_Tilter);
        SmartDashboard.putNumber(MotorName + " I Gain", kI_Tilter);
        SmartDashboard.putNumber(MotorName + " D Gain", kD_Tilter);
        SmartDashboard.putNumber(MotorName + " I Zone", kIz);
        SmartDashboard.putNumber(MotorName + " Feed Forward", kFF);

    }

    public void getEncoderData()
    {
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
      SmartDashboard.putNumber(MotorName + " Velocity", CurrentEncoderVelocity*60);
      
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

      
    //   if((p != kP_Tilter)) { configs.Slot1.kP = p; kP_Tilter = p; SetConfigToMotor(); }
    // if((i != kI_Tilter)) { configs.Slot1.kI = i; kI_Tilter = i; SetConfigToMotor(); }
    // if((d != kD_Tilter)) { configs.Slot1.kD = d; kD_Tilter = d; SetConfigToMotor(); }
      
    
    }

    public void SetSpeed(double thisspeed) {
        Motor_Controller.set(thisspeed);
      }
    
      TalonFXConfiguration configs = new TalonFXConfiguration();
      private void SetupMotorConfig() {
      

      configs.Slot1.kP = 40; // An error of 1 rotations results in 40 amps output
      configs.Slot1.kI = 0;
      configs.Slot1.kD = 2; // A change of 1 rotation per second results in 2 amps output
      // Peak output of 130 amps
      configs.TorqueCurrent.PeakForwardTorqueCurrent = 130;
      configs.TorqueCurrent.PeakReverseTorqueCurrent = 130;
      
      configs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
      configs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

      configs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.DeliveryHead.Tilt_maxValue;
      configs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.DeliveryHead.Tilt_minValue;



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
        Motor_Controller.setControl(m_torquePosition.withPosition(WantedEncoderValue));
        //MotorControllerPid.setReference(WantedEncoderValue, CANSparkBase.ControlType.kPosition);
      }

      public void setSetpointZero() {
        //enable();
        WantedEncoderValue = Constants.DeliveryHead.Tilt_Position_Zero;
        Motor_Controller.setControl(m_torquePosition.withPosition(WantedEncoderValue));
      }
        public void setSetpointPassing() {
        //enable();
        WantedEncoderValue = Constants.DeliveryHead.Tilt_Position_Passing;
        Motor_Controller.setControl(m_torquePosition.withPosition(WantedEncoderValue));
      }
      public void setSetpointAmp() {
        //enable();
        WantedEncoderValue = Constants.DeliveryHead.Tilt_Position_Amp;
        Motor_Controller.setControl(m_torquePosition.withPosition(WantedEncoderValue));
      }


      public boolean HasNote = false;


    double setpointTolerance = 2.0;
      public boolean atSetpoint() {        
          if (Constants.isWithinAmount(CurrentEncoderValue, WantedEncoderValue, setpointTolerance)) {
            return true;
          } else {
            return false; 
          }
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
