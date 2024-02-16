package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DeliveryShooter extends SubsystemBase {
    
    
    private static final String MotorName = "Upper Shooter";
    private static final String MotorName2 = "Lower Shooter";
    private CANSparkMax m_motor;
    private CANSparkMax m_motor_LowS;
    private SparkPIDController m_pidController;
    private SparkPIDController m_pidController_LowS;
    private RelativeEncoder m_encoder;
    private RelativeEncoder m_encoder_LowS;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;
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

    public DeliveryShooter()
    {
        m_motor = new CANSparkMax(Constants.CANBus.UppershooterCanID, MotorType.kBrushless);
        m_motor_LowS = new CANSparkMax(Constants.CANBus.LowerShooterCanID, MotorType.kBrushless);
        //m_motor_LowS.follow(m_motor, false);

        //set the idle mode to brake so it doesnt move when we dont want it to, or coast if we want it to coast after "stopping"
        m_motor.setIdleMode(IdleMode.kCoast);
        m_motor_LowS.setIdleMode(IdleMode.kCoast);
        //set the ramp rate to controll sudden input changes (smooth input
        m_motor.setClosedLoopRampRate(.1);
        m_motor.setOpenLoopRampRate(.1);//small ramp rate becuase this will reverse instantly.

        m_motor_LowS.setClosedLoopRampRate(.1);
        m_motor_LowS.setOpenLoopRampRate(.1);//small ramp rate becuase this will reverse instantly. 
        //current limit to keep motors safe from Fire (over current)
        m_motor.setSmartCurrentLimit(Constants.NeoBrushless.neo1650safelimitAmps);
        m_motor_LowS.setSmartCurrentLimit(Constants.NeoBrushless.neo1650safelimitAmps);
        /**
         * In order to use PID functionality for a controller, a SparkPIDController object
         * is constructed by calling the getPIDController() method on an existing
         * CANSparkMax object
         */
        m_pidController = m_motor.getPIDController();
        m_pidController_LowS = m_motor_LowS.getPIDController();
        // Encoder object created to display position values
        m_encoder = m_motor.getEncoder();
        m_encoder_LowS = m_motor_LowS.getEncoder();

        // PID coefficients
        kP = 0.000160; 
        kI = 0.000001;
        kD = 0.000001; 
        kIz = 0; 
        kFF = 0.000015; 
        kMaxOutput = 1; 
        kMinOutput = -1;
        maxRPM = 5700;

        // set PID coefficients
        m_pidController.setP(kP);
        m_pidController.setI(kI);
        m_pidController.setD(kD);
        m_pidController.setIZone(kIz);
        m_pidController.setFF(kFF);
        m_pidController.setOutputRange(kMinOutput, kMaxOutput);

        // set PID coefficients
        m_pidController_LowS.setP(kP);
        m_pidController_LowS.setI(kI);
        m_pidController_LowS.setD(kD);
        m_pidController_LowS.setIZone(kIz);
        m_pidController_LowS.setFF(kFF);
        m_pidController_LowS.setOutputRange(kMinOutput, kMaxOutput);

        // display PID coefficients on SmartDashboard
        SmartDashboard.putNumber(MotorName + " P Gain", kP);
        SmartDashboard.putNumber(MotorName + " I Gain", kI);
        SmartDashboard.putNumber(MotorName + " D Gain", kD);
        SmartDashboard.putNumber(MotorName + " I Zone", kIz);
        SmartDashboard.putNumber(MotorName + " Feed Forward", kFF);
        SmartDashboard.putNumber(MotorName + " Max Output", kMaxOutput);
        SmartDashboard.putNumber(MotorName + " Min Output", kMinOutput);
        SmartDashboard.putNumber(MotorName + " Setpoint RPM", WantedRPM);
    }

    @Override
    public void periodic()
    {
        getEncoderData();
        double p = SmartDashboard.getNumber(MotorName + " P Gain", 0);
        double i = SmartDashboard.getNumber(MotorName + " I Gain", 0);
        double d = SmartDashboard.getNumber(MotorName + " D Gain", 0);
        double iz = SmartDashboard.getNumber(MotorName + " I Zone", 0);
        double ff = SmartDashboard.getNumber(MotorName + " Feed Forward", 0);
        double WRPM = SmartDashboard.getNumber(MotorName + " Setpoint RPM", 0);
          
          if((p != kP)) { m_pidController.setP(p); m_pidController_LowS.setP(p); kP = p; }
        if((i != kI)) { m_pidController.setI(i); m_pidController_LowS.setI(i); kI = i; }
        if((d != kD)) { m_pidController.setD(d); m_pidController_LowS.setD(d); kD = d; }
        if((iz != kIz)) { m_pidController.setIZone(iz); m_pidController_LowS.setIZone(iz); kIz = iz; }
        if((ff != kFF)) { m_pidController.setFF(ff); m_pidController_LowS.setFF(ff); kFF = ff; }

        if (WRPM != WantedRPM) {
            WantedRPM = WRPM;
            m_pidController.setReference(WantedRPM, CANSparkMax.ControlType.kVelocity);
            m_pidController_LowS.setReference(WantedRPM, CANSparkMax.ControlType.kVelocity);
            
        }

        //SmartDashboard.putNumber("SetPoint RPM  UpS", WantedRPM);
        //SmartDashboard.putNumber("Current velocity UpS", m_encoder.getVelocity());
    }

    public void getEncoderData()
    {
      OutputCurrent = m_motor.getOutputCurrent();
      SmartDashboard.putNumber(MotorName + " Amps",OutputCurrent);
        
      OutputCurrent_LowS= m_motor_LowS.getOutputCurrent();
      SmartDashboard.putNumber(MotorName2 + " Amps",OutputCurrent_LowS);

      MotorTemp = m_motor.getMotorTemperature();
      SmartDashboard.putNumber(MotorName + " Motor Temp",MotorTemp);

      MotorTemp_LowS = m_motor_LowS.getMotorTemperature();
      SmartDashboard.putNumber(MotorName2 + " Motor Temp",MotorTemp_LowS);
      /**
       * Encoder position is read from a RelativeEncoder object by calling the
       * GetPosition() method.
       * 
       * GetPosition() returns the position of the encoder in units of revolutions
       */
      CurrentEncoderValue = m_encoder.getPosition();
      SmartDashboard.putNumber(MotorName + " PID Encoder Position",CurrentEncoderValue);

      CurrentEncoderValue_LowS = m_encoder_LowS.getPosition();
      SmartDashboard.putNumber(MotorName2 + " PID Encoder Position",CurrentEncoderValue_LowS);
  
      /**
       * Encoder velocity is read from a RelativeEncoder object by calling the
       * GetVelocity() method.
       * 
       * GetVelocity() returns the velocity of the encoder in units of RPM
       */
      CurrentEncoderVelocity = m_encoder.getVelocity();
      SmartDashboard.putNumber(MotorName + " Velocity", CurrentEncoderVelocity);

      CurrentEncoderVelocity_LowS = m_encoder_LowS.getVelocity();
      SmartDashboard.putNumber(MotorName2 + " Velocity", CurrentEncoderVelocity_LowS);
  
    }
}
