package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DeliveryShooter extends SubsystemBase {
    
    
    private static final String MotorName = "Upper Shooter";
    private static final String MotorName2 = "Lower Shooter";
    private TalonFX m_motor;
    private TalonFX m_motor_LowS;
    /* Start at velocity 0, no feed forward, use slot 1 */
    private final VelocityTorqueCurrentFOC m_torqueVelocity = new VelocityTorqueCurrentFOC(0, 0, 0, 1, true, false, false);
    /* Keep a neutral out so we can disable the motor */
    private final PositionTorqueCurrentFOC m_torquePosition = new PositionTorqueCurrentFOC(0, 0, 0, 1, true, false, false);

    private final NeutralOut m_brake = new NeutralOut();

    
    public double LastSetRPM = 0;
    public double WantedRPM = 0;
    public double WantedEncoderValue = 0;

    public double CurrentEncoderValue = 0;
    public double CurrentEncoderVelocity = 0;
    public double OutputCurrent = 0;
    public double MotorTemp = 0;

    public double CurrentEncoderValue_LowS = 0;
    public double CurrentEncoderVelocity_LowS = 0;
    public double OutputCurrent_LowS = 0;
    public double MotorTemp_LowS = 0;

    public double TempCForOverTemp = 37;


    public void SetShootSpeed(double rpm)
    {
      WantedRPM = rpm;
    }

    public double kP = 13;
    public double kI = .8;
    public double kD = .001;
    public DeliveryShooter()
    {
        

        m_motor = new TalonFX(Constants.CANBus.UppershooterCanID,Constants.CANBus.kRIOCANbusName);//new CANSparkMax(Constants.CANBus.UppershooterCanID, MotorType.kBrushless);
        m_motor_LowS = new TalonFX(Constants.CANBus.LowerShooterCanID,Constants.CANBus.kRIOCANbusName);//new CANSparkMax(Constants.CANBus.LowerShooterCanID, MotorType.kBrushless);
        
        SetupMotorConfig();


        
        

        // display PID coefficients on SmartDashboard
        SmartDashboard.putNumber(MotorName + " P Gain", kP);
        SmartDashboard.putNumber(MotorName + " I Gain", kI);
        SmartDashboard.putNumber(MotorName + " D Gain", kD);
        // SmartDashboard.putNumber(MotorName + " I Zone", kIz);
        // SmartDashboard.putNumber(MotorName + " Feed Forward", kFF);
        // SmartDashboard.putNumber(MotorName + " Max Output", kMaxOutput);
        // SmartDashboard.putNumber(MotorName + " Min Output", kMinOutput);
        // SmartDashboard.putNumber(MotorName + " Setpoint RPM", WantedRPM);
    }
    TalonFXConfiguration configs;
    private void SetupMotorConfig() {
       configs = new TalonFXConfiguration();
      /* Torque-based velocity does not require a feed forward, as torque will accelerate the rotor up to the desired velocity by itself */
      configs.Slot1.kP = kP;//5; // An error of 1 rotation per second results in 5 amps output
      configs.Slot1.kI = kI;//0.1; // An error of 1 rotation per second increases output by 0.1 amps every second
      configs.Slot1.kD = kD;//0.001; // A change of 1000 rotation per second squared results in 1 amp output

      // Peak output of 40 amps
      configs.TorqueCurrent.PeakForwardTorqueCurrent = 40;
      configs.TorqueCurrent.PeakReverseTorqueCurrent = -40;
      
      SetConfigToMotor();
    }

    @Override
    public void periodic()
    {
        getEncoderData();
        double p = SmartDashboard.getNumber(MotorName + " P Gain", 0);
        double i = SmartDashboard.getNumber(MotorName + " I Gain", 0);
        double d = SmartDashboard.getNumber(MotorName + " D Gain", 0);
          
        if((p != kP)) { configs.Slot1.kP = p; kP = p; SetConfigToMotor(); }
        if((i != kI)) { configs.Slot1.kI = i; kI = i; SetConfigToMotor(); }
        if((d != kD)) { configs.Slot1.kD = d; kD = d; SetConfigToMotor(); }

        if (LastSetRPM != WantedRPM) {
            LastSetRPM = WantedRPM;
            if(WantedRPM != 0)
            {
                double friction_torque = 0;// (WantedRPM > 0) ? 1 : -1; // To account for friction, we add this to the arbitrary feed forward
                /* Use torque velocity */
                m_motor.setControl(m_torqueVelocity.withVelocity(WantedRPM/60).withFeedForward(friction_torque));
                m_motor_LowS.setControl(m_torqueVelocity.withVelocity(WantedRPM/60).withFeedForward(friction_torque));
            }
            else
            {
               /* Disable the motor instead */
              m_motor.setControl(m_brake);
              m_motor_LowS.setControl(m_brake);
            }
            
            
        }

        SmartDashboard.putNumber(MotorName + "SetPoint RPM", WantedRPM);
        //SmartDashboard.putNumber("Current velocity UpS", m_encoder.getVelocity());
    }

    private void SetConfigToMotor() {
      /* Retry config apply up to 5 times, report if failure */
      StatusCode status = StatusCode.StatusCodeNotInitialized;
      for (int i = 0; i < 5; ++i) {
        //PUT MOTORS TO BE CONFIGED HERE
        status = m_motor.getConfigurator().apply(configs);
        status = m_motor_LowS.getConfigurator().apply(configs);
        //
        if (status.isOK()) break;
      }
      if(!status.isOK()) {
        System.out.println("Could not apply configs, error code: " + status.toString());
      }
    }

    public void MovePosition(double amount,boolean ZeropositionBeforemove)
    {
      if(ZeropositionBeforemove)
      {
          m_motor.setPosition(0);
          m_motor_LowS.setPosition(0);
      }
      WantedEncoderValue = amount;
        m_motor.setControl(m_torquePosition.withPosition(WantedEncoderValue));
        m_motor_LowS.setControl(m_torquePosition.withPosition(WantedEncoderValue));
    }

    public void resetSettleTimer()
    {
      m_SettleTimer.reset();
      m_SettleTimer.start();
    }
    double SettleTimeAtCorrectRPM = .05;
    public double RPMpercentageTolerance = 4;
    private final Timer m_SettleTimer = new Timer();
    public boolean getRPMReadyTofire(double RpmTolerance) {
      boolean isUpperWithinRange = Constants.isWithinPercentage(CurrentEncoderVelocity, LastSetRPM, RpmTolerance);
      boolean islowerWithinRange = Constants.isWithinPercentage(CurrentEncoderVelocity_LowS, LastSetRPM, RpmTolerance);
      SmartDashboard.putBoolean("isUpperWithinRange", isUpperWithinRange);
      SmartDashboard.putBoolean("islowerWithinRange", islowerWithinRange);
      boolean ReadyTofire = false;
      if (isUpperWithinRange && islowerWithinRange)
      {
        if(m_SettleTimer.get() > SettleTimeAtCorrectRPM)
        {
          ReadyTofire = true;
        }
      }
      else
      {
        resetSettleTimer();
      }
      return ReadyTofire;
    }
    public boolean getRPMReadyTofire() {
      return getRPMReadyTofire(RPMpercentageTolerance);
    }
    public void getEncoderData()
    {
      OutputCurrent = m_motor.getTorqueCurrent().getValueAsDouble();
      SmartDashboard.putNumber(MotorName + " Amps",OutputCurrent);
        
      OutputCurrent_LowS= m_motor_LowS.getTorqueCurrent().getValueAsDouble();
      SmartDashboard.putNumber(MotorName2 + " Amps",OutputCurrent_LowS);

      MotorTemp = m_motor.getDeviceTemp().getValueAsDouble();
      SmartDashboard.putNumber(MotorName + " Motor Temp",MotorTemp);

      MotorTemp_LowS = m_motor_LowS.getDeviceTemp().getValueAsDouble();
      SmartDashboard.putNumber(MotorName2 + " Motor Temp",MotorTemp_LowS);

      CurrentEncoderValue = m_motor.getPosition().getValueAsDouble();
      SmartDashboard.putNumber(MotorName + " PID Encoder Position",CurrentEncoderValue);

      CurrentEncoderValue_LowS = m_motor_LowS.getPosition().getValueAsDouble();
      SmartDashboard.putNumber(MotorName2 + " PID Encoder Position",CurrentEncoderValue_LowS);
  


      double vel_RotPerMin = m_motor.getRotorVelocity().getValueAsDouble() * 60.0;
      double vel_RotPerMin_LowS = m_motor_LowS.getRotorVelocity().getValueAsDouble() * 60.0;

      CurrentEncoderVelocity = vel_RotPerMin;
      SmartDashboard.putNumber(MotorName + " Rpm", CurrentEncoderVelocity);

      CurrentEncoderVelocity_LowS = vel_RotPerMin_LowS;
      SmartDashboard.putNumber(MotorName2 + " Rpm", CurrentEncoderVelocity_LowS);


    }
}
