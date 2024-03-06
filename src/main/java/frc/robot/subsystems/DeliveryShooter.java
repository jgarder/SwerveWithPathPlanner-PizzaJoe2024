package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DeliveryShooter extends SubsystemBase {
    
    
    private static final String MotorName = "Upper Shooter";
    private static final String MotorName2 = "Lower Shooter";
    private TalonFX m_motor;
    private TalonFX m_motor_LowS;
    /* Start at velocity 0, enable FOC, no feed forward, use slot 0 */
    private final VelocityVoltage m_voltageVelocity = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);
    /* Start at velocity 0, no feed forward, use slot 1 */
    private final VelocityTorqueCurrentFOC m_torqueVelocity = new VelocityTorqueCurrentFOC(0, 0, 0, 1, true, false, false);
    /* Keep a neutral out so we can disable the motor */
    private final NeutralOut m_brake = new NeutralOut();

    public double maxRPM;
    public double LastSetRPM = 0;
    public double WantedRPM = 0;

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

    public DeliveryShooter()
    {
        TalonFXConfiguration configs = new TalonFXConfiguration();

        m_motor = new TalonFX(Constants.CANBus.UppershooterCanID,Constants.CANBus.kRIOCANbusName);//new CANSparkMax(Constants.CANBus.UppershooterCanID, MotorType.kBrushless);
        m_motor_LowS = new TalonFX(Constants.CANBus.LowerShooterCanID,Constants.CANBus.kRIOCANbusName);//new CANSparkMax(Constants.CANBus.LowerShooterCanID, MotorType.kBrushless);
         /* Voltage-based velocity requires a feed forward to account for the back-emf of the motor */
        configs.Slot0.kP = 0.11; // An error of 1 rotation per second results in 2V output
        configs.Slot0.kI = 0.5; // An error of 1 rotation per second increases output by 0.5V every second
        configs.Slot0.kD = 0.0001; // A change of 1 rotation per second squared results in 0.01 volts output
        configs.Slot0.kV = 0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second
        // Peak output of 8 volts
        configs.Voltage.PeakForwardVoltage = 8;
        configs.Voltage.PeakReverseVoltage = -8;
        
        /* Torque-based velocity does not require a feed forward, as torque will accelerate the rotor up to the desired velocity by itself */
        configs.Slot1.kP = 5;//5; // An error of 1 rotation per second results in 5 amps output
        configs.Slot1.kI = .3;//0.1; // An error of 1 rotation per second increases output by 0.1 amps every second
        configs.Slot1.kD = .001;//0.001; // A change of 1000 rotation per second squared results in 1 amp output

        // Peak output of 40 amps
        configs.TorqueCurrent.PeakForwardTorqueCurrent = 40;
        configs.TorqueCurrent.PeakReverseTorqueCurrent = -40;
        
        /* Retry config apply up to 5 times, report if failure */
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
          status = m_motor.getConfigurator().apply(configs);
          status = m_motor_LowS.getConfigurator().apply(configs);
          if (status.isOK()) break;
        }
        if(!status.isOK()) {
          System.out.println("Could not apply configs, error code: " + status.toString());
        }

        //m_motor_LowS.setControl(new Follower(m_motor.getDeviceID(), false));

        maxRPM = 6250;
        

        // display PID coefficients on SmartDashboard
        // SmartDashboard.putNumber(MotorName + " P Gain", kP);
        // SmartDashboard.putNumber(MotorName + " I Gain", kI);
        // SmartDashboard.putNumber(MotorName + " D Gain", kD);
        // SmartDashboard.putNumber(MotorName + " I Zone", kIz);
        // SmartDashboard.putNumber(MotorName + " Feed Forward", kFF);
        // SmartDashboard.putNumber(MotorName + " Max Output", kMaxOutput);
        // SmartDashboard.putNumber(MotorName + " Min Output", kMinOutput);
        // SmartDashboard.putNumber(MotorName + " Setpoint RPM", WantedRPM);
    }

    @Override
    public void periodic()
    {
        getEncoderData();
        // double p = SmartDashboard.getNumber(MotorName + " P Gain", 0);
        // double i = SmartDashboard.getNumber(MotorName + " I Gain", 0);
        // double d = SmartDashboard.getNumber(MotorName + " D Gain", 0);
        // double iz = SmartDashboard.getNumber(MotorName + " I Zone", 0);
        // double ff = SmartDashboard.getNumber(MotorName + " Feed Forward", 0);
        // //double WRPM = SmartDashboard.getNumber(MotorName + " Setpoint RPM", 0);
          
        //   if((p != kP)) { m_pidController.setP(p); m_pidController_LowS.setP(p); kP = p; }
        // if((i != kI)) { m_pidController.setI(i); m_pidController_LowS.setI(i); kI = i; }
        // if((d != kD)) { m_pidController.setD(d); m_pidController_LowS.setD(d); kD = d; }
        // if((iz != kIz)) { m_pidController.setIZone(iz); m_pidController_LowS.setIZone(iz); kIz = iz; }
        // if((ff != kFF)) { m_pidController.setFF(ff); m_pidController_LowS.setFF(ff); kFF = ff; }

        if (LastSetRPM != WantedRPM) {
            LastSetRPM = WantedRPM;
            if(WantedRPM != 0)
            {
                double friction_torque =0;// (WantedRPM > 0) ? 1 : -1; // To account for friction, we add this to the arbitrary feed forward
                /* Use torque velocity */
                m_motor.setControl(m_torqueVelocity.withVelocity(WantedRPM/60).withFeedForward(friction_torque));
                m_motor_LowS.setControl(m_torqueVelocity.withVelocity(WantedRPM/60).withFeedForward(friction_torque));
                // /* Use voltage velocity */
                 //m_motor.setControl(m_voltageVelocity.withVelocity(WantedRPM/60));
            }
            else
            {
               /* Disable the motor instead */
              m_motor.setControl(m_brake);
              m_motor_LowS.setControl(m_brake);
            }
            
            
        }

        //SmartDashboard.putNumber("SetPoint RPM  UpS", WantedRPM);
        //SmartDashboard.putNumber("Current velocity UpS", m_encoder.getVelocity());
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
