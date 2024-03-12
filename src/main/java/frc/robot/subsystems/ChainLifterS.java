package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
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

public class ChainLifterS extends SubsystemBase {

    
    public String MotorName = "ChainLifter";
    public double CurrentEncoderValue = 0;
    public double WantedEncoderValue = 0;
    public double CurrentEncoderVelocity = 0;
    public double OutputCurrent = 0;
    public double MotorTemp = 0;
    public double TempCForOverTemp = 37;
    double kMaxOutput = 1; 
    double kMinOutput = -1;

    double kP_lifter = 6.000;//0.0400;
    double kI_lifter = 0.000004;
    double kD_lifter = 0.200001;

    double kFF = 0.00;
    double kIz = 0;

    TalonFXConfiguration configs = new TalonFXConfiguration();
    private final TalonFX Motor_Controller = new TalonFX(Constants.CANBus.ChainLifterCanBusID,Constants.CANBus.kRIOCANbusName);
      /* Start at position 0, no feed forward, use slot 1 */
    private final PositionTorqueCurrentFOC m_torquePosition = new PositionTorqueCurrentFOC(0, 0, 0, 1, false, false, false);
    /* Keep a neutral out so we can disable the motor */
    private final NeutralOut m_brake = new NeutralOut();

   // private final CANSparkMax Motor_Controller = new CANSparkMax(Constants.CANBus.ChainLifterCanBusID, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
   // private final RelativeEncoder Motor_Encoder = Motor_Controller.getEncoder();
   // private final SparkPIDController MotorControllerPid = Motor_Controller.getPIDController();
    
    public ChainLifterS()
    {

      SetupMotorConfig();
      Motor_Controller.setPosition(0);
      //should the motor controller be inverted? 0 is folded in and 44 (or max) is folded out.
      Motor_Controller.setInverted(false);
      Motor_Controller.setNeutralMode(NeutralModeValue.Brake);
      // display PID coefficients on SmartDashboard
      SmartDashboard.putNumber(MotorName + " P Gain", kP_lifter);
      SmartDashboard.putNumber(MotorName + " I Gain", kI_lifter);
      SmartDashboard.putNumber(MotorName + " D Gain", kD_lifter);

    }
    
      private void SetupMotorConfig() {
      

      configs.Slot1.kP = kP_lifter; // An error of 1 rotations results in 40 amps output
      configs.Slot1.kI = kI_lifter;
      configs.Slot1.kD = kD_lifter; // A change of 1 rotation per second results in 2 amps output
      // Peak output of 130 amps
      configs.TorqueCurrent.PeakForwardTorqueCurrent = 40;
      configs.TorqueCurrent.PeakReverseTorqueCurrent = -40;
      
      configs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
      configs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

      configs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.ChainLifter.Lift_maxValue;
      configs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.ChainLifter.Lift_minValue;

      

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
      
      public void disableatpark()
      {
        setSetpointToPosition(CurrentEncoderValue);
        //Motor_Controller.stopMotor();
        Motor_Controller.setControl(m_brake);//we press into our hysterisis on powerup. so without this the tilt motor always runs trying to go to the bottom. 
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

      
      if((p != kP_lifter)) { configs.Slot1.kP = p; kP_lifter = p; SetConfigToMotor(); }
      if((i != kI_lifter)) { configs.Slot1.kI = i; kI_lifter = i; SetConfigToMotor(); }
      if((d != kD_lifter)) { configs.Slot1.kD = d; kD_lifter = d; SetConfigToMotor(); }
    
    }

    public void SetSpeed(double thisspeed) {
        Motor_Controller.set(thisspeed);
      }
    


      public void AlterSetpointposition(double AddToPosition)
      {
        WantedEncoderValue = WantedEncoderValue + AddToPosition;
        Motor_Controller.setControl(m_torquePosition.withPosition(WantedEncoderValue));
      }

      public void setSetpointToPosition(double position)
      {
        //enable();
        WantedEncoderValue = position;
        Motor_Controller.setControl(m_torquePosition.withPosition(WantedEncoderValue));
      }

      public void setSetpointZero() {
        //enable();
        WantedEncoderValue = Constants.ChainLifter.Lift_Position_Zero;
        Motor_Controller.setControl(m_torquePosition.withPosition(WantedEncoderValue));
      }
        public void setSetpointCenterAndTrapLift() {
        //enable();
        WantedEncoderValue = Constants.ChainLifter.Lift_Position_CenterAndTrap;
        Motor_Controller.setControl(m_torquePosition.withPosition(WantedEncoderValue));
      }


      public boolean HasNote = false;


    double setpointTolerance = 1.0;
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
