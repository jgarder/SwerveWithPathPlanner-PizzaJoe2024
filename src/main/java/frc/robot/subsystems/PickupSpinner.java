package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;

import frc.robot.Constants;
import frc.robot.RobotContainer.PizzaManager;

public class PickupSpinner extends SubsystemBase{

    public String MotorName = "PickUp Intake";
    public double CurrentEncoderValue = 0;
    public double CurrentEncoderVelocity = 0;
    public double OutputCurrent = 0;
    public double MotorTemp = 0;
    public double TempCForOverTemp = 37;

    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

    public double idledutycycle = 0;
    public double PostrollDutyCycle = -0.55;//-0.3;//-.1;
    public double IntakeDutyCycle = -1.0;//rpm used during note release
    public double PassingDutyCycle = .75;//rpm used during note release
    public double LastSetRPM = 0;

    //boolean NoteInPickup = false;

    private final CANSparkMax Motor_Controller = new CANSparkMax(Constants.CANBus.PickUpSpinnerCanBusID, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
    private final RelativeEncoder Motor_Encoder = Motor_Controller.getEncoder();
    private final SparkPIDController MotorControllerPid = Motor_Controller.getPIDController();
    public SparkLimitSwitch m_forwardLimit = Motor_Controller.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
    
    public boolean getLimitSwitchEnabled()
    {
      return m_forwardLimit.isPressed();
    }
    
    public PickupSpinner()
    {
        //super(new PIDController(Constants.PickupHead.kP_Spinner, Constants.PickupHead.kI_Spinner, Constants.PickupHead.kD_Spinner));//super class, must setup PID first
         //even though the default should be 0, lets tell the PID to goto 0 which is our starting position.
         Motor_Encoder.setPosition(Constants.PizzaFloorPickupHead.minValue_Spinner);
         //setSetpoint(0);

         //the forward limit switch is used to detect note in pickup.
         m_forwardLimit.enableLimitSwitch(true);

        //set the idle mode to brake so it doesnt move when we dont want it to, or coast if we want it to coast after "stopping"
        Motor_Controller.setIdleMode(IdleMode.kBrake);
        
        //motor is inverted in our situation. negative is sucking in, positive is shooting out. 
        Motor_Controller.setInverted(true);

        //set the ramp rate to controll sudden input changes (smooth input
        Motor_Controller.setClosedLoopRampRate(.05);
        Motor_Controller.setOpenLoopRampRate(.05);//small ramp rate becuase this will reverse instantly. 
        
        //current limit to keep motors safe from Fire (over current)
        Motor_Controller.setSmartCurrentLimit(Constants.NeoBrushless.neo550safelimitAmps);

        // set voltage sag compensation. 
        Motor_Controller.enableVoltageCompensation(Constants.nominalBatteryVoltage);

        //limit everything on this motor controller to 500ms except the status 0 frame which is 10ms and does faults and applied output. 
        // Motor_Controller.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);  //Default Rate: 20ms ,Motor Velocity,Motor Temperature,Motor VoltageMotor Current
        // Motor_Controller.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);  //Default Rate: 20ms ,Motor Position
        // Motor_Controller.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500); //Default Rate: 50ms ,Analog Sensor Voltage ,Analog Sensor Velocity ,Analog Sensor Position
        // Motor_Controller.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 500); //Default Rate: 20ms, Alternate Encoder Velocity,Alternate Encoder Position
        // Motor_Controller.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 500); //Default Rate: 200ms, Duty Cycle Absolute Encoder Position,Duty Cycle Absolute Encoder Absolute Angle
        // Motor_Controller.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 500); //Default Rate: 200ms, Duty Cycle Absolute Encoder Velocity,Duty Cycle Absolute Encoder Frequency
        // Motor_Controller.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10);  //Default Rate: 20ms ,Motor Velocity,Motor Temperature,Motor VoltageMotor Current
        // Motor_Controller.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);  //Default Rate: 20ms ,Motor Velocity,Motor Temperature,Motor VoltageMotor Current
        // Motor_Controller.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);  //Default Rate: 20ms ,Motor Position
        // Motor_Controller.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 50); //Default Rate: 50ms ,Analog Sensor Voltage ,Analog Sensor Velocity ,Analog Sensor Position
        // Motor_Controller.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 20); //Default Rate: 20ms, Alternate Encoder Velocity,Alternate Encoder Position
        // Motor_Controller.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 500); //Default Rate: 200ms, Duty Cycle Absolute Encoder Position,Duty Cycle Absolute Encoder Absolute Angle
        // Motor_Controller.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 500); //Default Rate: 200ms, Duty Cycle Absolute Encoder Velocity,Duty Cycle Absolute Encoder Frequency

        SmartDashboard.putBoolean(MotorName + " Forward Limit Enabled", m_forwardLimit.isLimitSwitchEnabled());

        // PID coefficients
        kP = 0.000074; 
        kI = 0.00000051;
        kD = 0.000000; 
        kIz = 0; 
        kFF = 0.000008; 
        kMaxOutput = 1; 
        kMinOutput = -1;

        // set PID coefficients
        //MotorControllerPid.setP(kP);
        //MotorControllerPid.setI(kI);
        //MotorControllerPid.setD(kD);
        //MotorControllerPid.setIZone(kIz);
        MotorControllerPid.setFF(kFF);
        MotorControllerPid.setOutputRange(kMinOutput, kMaxOutput);

        // display PID coefficients on SmartDashboard
        //SmartDashboard.putNumber(MotorName + " P Gain", kP);
        //SmartDashboard.putNumber(MotorName + " I Gain", kI);
        //SmartDashboard.putNumber(MotorName + " D Gain", kD);
        //SmartDashboard.putNumber(MotorName + " I Zone", kIz);
        //SmartDashboard.putNumber(MotorName + " Feed Forward", kFF);
        //SmartDashboard.putBoolean(MotorName + " Forward Limit Enabled", m_forwardLimit.isLimitSwitchEnabled());

        //enable();//enable the pidcontroller of this subsystem 
    }
    
    public boolean setIsnoteInPickup(boolean isnoteinpickup)
    {
      PizzaManager.IsNoteInPickup = isnoteinpickup;
      return isnoteinpickup;
    }
    public boolean IsNoteInPickup()
    {
      return PizzaManager.IsNoteInPickup;
    }
    public void SetSpeed(double thisspeed) {
        Motor_Controller.set(thisspeed);
    }

    public void RunPickup() {
      boolean isenabled = SmartDashboard.getBoolean(MotorName + " Forward Limit Enabled", true);
      m_forwardLimit.enableLimitSwitch(isenabled);
      //m_forwardLimit.enableLimitSwitch(true);
        
        //if our limit switch is turned on And there is no note in the pickup, then we can run the pickup motor.. 
        if (isenabled) 
        {
            boolean istriggered = m_forwardLimit.isPressed();
            if (istriggered) {//we have a note in pickup and limit switch is pressed, so we need to reduce the setpoint to hold the note in pickup.
                if (!PizzaManager.IsNoteInPickup) {
                    //HoldAutoLoaded();
                    stopSpinner();
                PizzaManager.IsNoteInPickup = true;
                return;
                }
                else
                {
                    PizzaManager.IsNoteInPickup = true;
                    stopSpinner();
                    //trying to run pickup but the limit switch is enabled and there is a note in the pickup so dont run the head. 
                }
                return;   
            }   
      
        }
        IntakeRunCommand();
    }
    //int reduction = 15;
    public void IntakeRunCommand() {
      //m_forwardLimit.enableLimitSwitch(false);
      SetToWantedRpm(IntakeDutyCycle);
      //Motor_Encoder.setPosition(0);
      //setSetpoint(-reduction);
      //enable();
    }
        public void IntakePostRollCommand() {
      //m_forwardLimit.enableLimitSwitch(false);
      SetToWantedRpm(PostrollDutyCycle);
      //Motor_Encoder.setPosition(0);
      //setSetpoint(-reduction);
      //enable();
    }
    //
    double windback = 3;
    public void HoldAutoLoaded(){
        //pickupState = pickupState.ZERO;
       // double position = Motor_Encoder.getPosition();
       // Motor_Encoder.setPosition(position);
        SetToWantedRpm(idledutycycle);
    }

    public void ReleaseNote(){
        m_forwardLimit.enableLimitSwitch(false);//so the note will shoot out even though we are endstopped (does it need this?)
        setIsnoteInPickup(false);
        SetToWantedRpm(PassingDutyCycle);
        
    }

    private void SetToWantedRpm(double wantedRPM) {
      if (wantedRPM != LastSetRPM) {
        LastSetRPM = wantedRPM;
        MotorControllerPid.setReference(LastSetRPM, CANSparkBase.ControlType.kDutyCycle);
      }
    }
    
    public void stopSpinner()
    {
      LastSetRPM = 0;
      MotorControllerPid.setReference(LastSetRPM, CANSparkBase.ControlType.kDutyCycle);
    }

    public void getEncoderData()
      {
        OutputCurrent = Motor_Controller.getOutputCurrent();
        SmartDashboard.putNumber(MotorName + " Amps",OutputCurrent);
    
        MotorTemp = Motor_Controller.getMotorTemperature();
        SmartDashboard.putNumber(MotorName + " Motor Temp",MotorTemp);
        /**
         * Encoder position is read from a RelativeEncoder object by calling the
         * GetPosition() method.
         * 
         * GetPosition() returns the position of the encoder in units of revolutions
         */
        CurrentEncoderValue = Motor_Encoder.getPosition();
        SmartDashboard.putNumber(MotorName + " PID Encoder Position",CurrentEncoderValue);
    
        /**
         * Encoder velocity is read from a RelativeEncoder object by calling the
         * GetVelocity() method.
         * 
         * GetVelocity() returns the velocity of the encoder in units of RPM
         */
        CurrentEncoderVelocity = Motor_Encoder.getVelocity();
        SmartDashboard.putNumber(MotorName + " Velocity", CurrentEncoderVelocity);
        
        //SmartDashboard.putBoolean(MotorName + " Forward Limit Enabled", m_forwardLimit.isLimitSwitchEnabled());//dont set this every loop because we want the user to have some control. 
        
      }
    @Override
    public void periodic() {
      getEncoderData();
         // enable/disable limit switches based on value read from SmartDashboard
    SmartDashboard.putBoolean(MotorName + " Forward Limit Triggered", m_forwardLimit.isPressed());

      PidTune(MotorName);

    super.periodic();// This is a PidSubsystem, we have orridden the periodic method to get encoder data... So we need to call the super periodic method to get the PID stuff to work.


  }

    private void PidTune(String PidName) {
      double p = SmartDashboard.getNumber(PidName + " P Gain", 0);
      double i = SmartDashboard.getNumber(PidName + " I Gain", 0);
      double d = SmartDashboard.getNumber(PidName + " D Gain", 0);
      //double iz = SmartDashboard.getNumber(PidName + " I Zone", 0);
      //double ff = SmartDashboard.getNumber(PidName + " Feed Forward", 0);
        
      if((p != kP)) { MotorControllerPid.setP(p); kP = p; }
      if((i != kI)) { MotorControllerPid.setI(i); kI = i; }
      if((d != kD)) { MotorControllerPid.setD(d); kD = d; }
      //if((iz != kIz)) { MotorControllerPid.setIZone(iz); kIz = iz; }
      //if((ff != kFF)) { MotorControllerPid.setFF(ff); kFF = ff; }
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
