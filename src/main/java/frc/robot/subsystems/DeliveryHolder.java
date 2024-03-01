package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer.PizzaManager;
import frc.robot.RobotContainer.PizzaManager.PizzaTracker;

public class DeliveryHolder extends SubsystemBase {

    
    public String MotorName = "Delivery Holder";
    public double CurrentEncoderValue = 0;
    public double WantedEncoderValue = 0;
    public double CurrentEncoderVelocity = 0;
    public double OutputCurrent = 0;
    public double MotorTemp = 0;
    public double TempCForOverTemp = 37;
    double kMaxOutput = 1; 
    double kMinOutput = -1;

    double kP_lifter = 0.0250;
    double kI_lifter = 0.0000;
    double kD_lifter = 0.000;

    double kFF = 0.0003;
    double kIz = 0;
    private final CANSparkMax Motor_Controller = new CANSparkMax(Constants.CANBus.DeliveryIntakeCanBusID, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
    private final RelativeEncoder Motor_Encoder = Motor_Controller.getEncoder();
    private final SparkPIDController MotorControllerPid = Motor_Controller.getPIDController();
    public final SparkLimitSwitch m_forwardLimit = Motor_Controller.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
    public DeliveryShooter ThisShooter;
    public DeliveryHolder(DeliveryShooter thisShooter)
    {
      ThisShooter = thisShooter;
        //super(new PIDController(Constants.PickupHead.kP_lifter, Constants.PickupHead.kI_lifter, Constants.PickupHead.kD_lifter));//super class, must setup PID first
        //even though the default should be 0, lets tell the PID to goto 0 which is our starting position.

        Motor_Encoder.setPosition(Constants.DeliveryHead.Lift_minValue);
        MotorControllerPid.setP(kP_lifter);
        MotorControllerPid.setI(kI_lifter);
        MotorControllerPid.setD(kD_lifter);
        MotorControllerPid.setFF(kFF);
        MotorControllerPid.setOutputRange(kMinOutput, kMaxOutput);
        MotorControllerPid.setReference(0.0, CANSparkBase.ControlType.kPosition);
        //setSetpoint(0);
        //should the motor controller be inverted? 0 is folded in and 44 (or max) is folded out.
        Motor_Controller.setInverted(true);

         //the forward limit switch is used to detect note in pickup.
         m_forwardLimit.enableLimitSwitch(true);

        //Enable the soft limits and set the values
        Motor_Controller.enableSoftLimit(SoftLimitDirection.kForward, false);
        Motor_Controller.enableSoftLimit(SoftLimitDirection.kReverse, false);
       // Motor_Controller.setSoftLimit(SoftLimitDirection.kForward, (float)Constants.DeliveryHead.Lift_maxValue);
        //Motor_Controller.setSoftLimit(SoftLimitDirection.kReverse, (float)Constants.DeliveryHead.Lift_minValue);

        //set the idle mode to brake so it doesnt move when we dont want it to, or coast if we want it to coast after "stopping"
        Motor_Controller.setIdleMode(IdleMode.kBrake);
        
        //set the ramp rate to controll sudden input changes (smooth input
        Motor_Controller.setClosedLoopRampRate(.01);
        Motor_Controller.setOpenLoopRampRate(.01);//small ramp rate becuase this will reverse instantly. 
        
        //current limit to keep motors safe from Fire (over current)
        Motor_Controller.setSmartCurrentLimit(Constants.NeoBrushless.neo1650safelimitAmps);

        //limit everything on this motor controller to 500ms except the status 0 frame which is 10ms and does faults and applied output. 
        // Motor_Controller.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);  //Default Rate: 20ms ,Motor Velocity,Motor Temperature,Motor VoltageMotor Current
        // Motor_Controller.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);  //Default Rate: 20ms ,Motor Position
        // Motor_Controller.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 250); //Default Rate: 50ms ,Analog Sensor Voltage ,Analog Sensor Velocity ,Analog Sensor Position
        // Motor_Controller.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 250); //Default Rate: 20ms, Alternate Encoder Velocity,Alternate Encoder Position
        // Motor_Controller.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 500); //Default Rate: 200ms, Duty Cycle Absolute Encoder Position,Duty Cycle Absolute Encoder Absolute Angle
        // Motor_Controller.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 500); //Default Rate: 200ms, Duty Cycle Absolute Encoder Velocity,Duty Cycle Absolute Encoder Frequency
        // Motor_Controller.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10);  //Default Rate: 20ms ,Motor Velocity,Motor Temperature,Motor VoltageMotor Current
        // Motor_Controller.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);  //Default Rate: 20ms ,Motor Velocity,Motor Temperature,Motor VoltageMotor Current
        // Motor_Controller.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);  //Default Rate: 20ms ,Motor Position
        // Motor_Controller.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 50); //Default Rate: 50ms ,Analog Sensor Voltage ,Analog Sensor Velocity ,Analog Sensor Position
        // Motor_Controller.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 20); //Default Rate: 20ms, Alternate Encoder Velocity,Alternate Encoder Position
        // Motor_Controller.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 500); //Default Rate: 200ms, Duty Cycle Absolute Encoder Position,Duty Cycle Absolute Encoder Absolute Angle
        // Motor_Controller.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 500); //Default Rate: 200ms, Duty Cycle Absolute Encoder Velocity,Duty Cycle Absolute Encoder Frequency
  
        
        // display PID coefficients on SmartDashboard
        SmartDashboard.putNumber(MotorName + " P Gain", kP_lifter);
        SmartDashboard.putNumber(MotorName + " I Gain", kI_lifter);
        SmartDashboard.putNumber(MotorName + " D Gain", kD_lifter);
        SmartDashboard.putNumber(MotorName + " I Zone", kIz);
        SmartDashboard.putNumber(MotorName + " Feed Forward", kFF);
        SmartDashboard.putBoolean(MotorName + " Forward Limit Enabled", m_forwardLimit.isLimitSwitchEnabled());
    }

    //int reduction = 15;
    public void IntakeRuntoHoldCommand(double reduction,boolean IgnorelimitSwitch) {
      if (IgnorelimitSwitch) {
        m_forwardLimit.enableLimitSwitch(false);
      }
      else
      {
        // m_forwardLimit.enableLimitSwitch(true);
      }
      //m_forwardLimit.enableLimitSwitch(false);
      boolean istriggered = m_forwardLimit.isPressed();
      
      if (istriggered & !IgnorelimitSwitch) {
        Motor_Encoder.setPosition(0);
        WantedEncoderValue = 0;
        MotorControllerPid.setReference(0, CANSparkBase.ControlType.kPosition);
        PizzaManager.NoteInDeliveryHolder = true;
        if(PizzaManager.pizzaStage == PizzaTracker.passed)//if we are first getting this note from the pass
          {
              PizzaManager.pizzaStage = PizzaTracker.indexNeeded;//then we need to index it
          }
        else if(PizzaManager.pizzaStage == PizzaTracker.NoteOutDexed)//if we are receiving from an outindex
          {
              PizzaManager.pizzaStage = PizzaTracker.Indexed;//then we are indexed
          }
      }
      else 
      {
        Motor_Encoder.setPosition(0);
        WantedEncoderValue = reduction;
        MotorControllerPid.setReference(reduction, CANSparkBase.ControlType.kPosition);
        PizzaManager.NoteInDeliveryHolder = false;
        //PizzaManager.pizzaStage = PizzaTracker.indexNeeded;
      }

      //enable();
    }
    public boolean requestingIndex = false;
    private Timer m_Timer = new Timer();
    public void RequestIndex()
    {
      m_Timer.reset();
      m_Timer.start();
      requestingIndex = true;
      SmartDashboard.putString("IndexStage","Requested");
      pickedupNote = false;
    }
    boolean pickedupNote = false;
    public void indexNote()
    {
      
       if(m_Timer.get() > 30){
        requestingIndex = false;
        m_forwardLimit.enableLimitSwitch(true);
        stopSpinner();
         return;
       } 
        if (requestingIndex) {
          if(PizzaManager.pizzaStage == PizzaTracker.passed)
          {
            //System.out.println("intaking");
            SmartDashboard.putString("IndexStage","Intaking");
              IntakeRuntoHoldCommand(40,false);
          }
          else if(PizzaManager.pizzaStage == PizzaTracker.indexNeeded)
          {
            m_forwardLimit.enableLimitSwitch(false);
            //System.out.println("outdexiong");
            SmartDashboard.putString("IndexStage","OutDexing");
            SetToWantedDutyCycle(-.3);
            
            isUndoIndexFinished();
          }
          else if(PizzaManager.pizzaStage == PizzaTracker.intaking)
          {
            m_forwardLimit.enableLimitSwitch(false);
            //System.out.println("FrontIntaking");
            SmartDashboard.putString("IndexStage","FrontIntaking");
            SetToWantedDutyCycle(-.15);//-.2 was working
            ThisShooter.SetShootSpeed(Constants.DeliveryHead.ShooterRpmHumanSource);
            if(IsNoteInDeliveryHold())
            {
              pickedupNote = true;
            }
            if(pickedupNote)
            {
              isUndoIndexFinished();
            }
          }
          else if(PizzaManager.pizzaStage == PizzaTracker.NoteOutDexed)
          {
            //System.out.println("secondindexing");
            SmartDashboard.putString("IndexStage","secondIndexnow");
              IntakeRuntoHoldCommand(40,false);
          }
          else if (PizzaManager.pizzaStage == PizzaTracker.Indexed) {
            MovePosition(indexBacklash);
            //System.out.println("Now indexed");
            SmartDashboard.putString("IndexStage","Indexed");
            requestingIndex = false;
          }
        }
        else{}
    }
    public double indexBacklash =-3.5;
    //double Timeout = .5;
  public boolean isUndoIndexFinished() {
    if (!IsNoteInDeliveryHold() ) {
      //System.out.println("No Note In Holder");
        PizzaManager.pizzaStage = PizzaTracker.NoteOutDexed;
        m_forwardLimit.enableLimitSwitch(true);
        if (DriverStation.isTeleopEnabled())//the autonomous preempts the RPM for shots and indexes. complicated but this hack works.  
        {
          ThisShooter.SetShootSpeed(Constants.DeliveryHead.ShooterRpmOff);
        }
        
        stopSpinner();
        return true;
      }  
    return false;
  }
  public void forceCancelIndex()
  {
      requestingIndex = false;
      m_forwardLimit.enableLimitSwitch(true);
      ThisShooter.SetShootSpeed(Constants.DeliveryHead.ShooterRpmOff);
      stopSpinner();
  }


    public void MovePosition(double amount)
    {
      Motor_Encoder.setPosition(0);
      WantedEncoderValue = amount;
        MotorControllerPid.setReference(amount, CANSparkBase.ControlType.kPosition);
    }

      public boolean IsNoteInDeliveryHold()
      {
        PizzaManager.NoteInDeliveryHolder = m_forwardLimit.isPressed();
        return PizzaManager.NoteInDeliveryHolder;
      }

      public double IdleDutyCycle = 0;
      public double SpeakerDutyCycle = 1;//rpm used during note release
      public double AmpDutyCycle = .75;//rpm used during note release
      public double PassingDutyCycle = .75;//rpm used during note release
      public double LastSetDutyCycle = 0;

      public void SetToWantedDutyCycle(double wantedDutyCycle) {
        if (wantedDutyCycle != LastSetDutyCycle) {
          LastSetDutyCycle = wantedDutyCycle;
          MotorControllerPid.setReference(LastSetDutyCycle, CANSparkBase.ControlType.kDutyCycle);
        }
      }
      
      public void stopSpinner()
      {
        LastSetDutyCycle = IdleDutyCycle;
        MotorControllerPid.setReference(LastSetDutyCycle, CANSparkBase.ControlType.kDutyCycle);
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
      
      SmartDashboard.putNumber(MotorName + " PID output",Motor_Controller.getAppliedOutput());
      SmartDashboard.putNumber(MotorName + " setpoint ",  WantedEncoderValue);
      

    }

    @Override
    public void periodic() {
      SmartDashboard.putBoolean(MotorName + " Forward Limit Triggered", m_forwardLimit.isPressed());
          //anything you wanted to do periodically put it here.
        //INSIDE GET ENCODER DATA WE UPDATE OUR CurrentLiftEncoderValue! this is how the PID WORKS!!!!
        getEncoderData();
        //super.periodic();// This is a PidSubsystem, we have orridden the periodic method to get encoder data... So we need to call the super periodic method to get the PID stuff to work.
        // read PID coefficients from SmartDashboard
      double p = SmartDashboard.getNumber(MotorName + " P Gain", 0);
      double i = SmartDashboard.getNumber(MotorName + " I Gain", 0);
      double d = SmartDashboard.getNumber(MotorName + " D Gain", 0);
      double iz = SmartDashboard.getNumber(MotorName + " I Zone", 0);
      double ff = SmartDashboard.getNumber(MotorName + " Feed Forward", 0);
        
        if((p != kP_lifter)) { MotorControllerPid.setP(p); kP_lifter = p; }
      if((i != kI_lifter)) { MotorControllerPid.setI(i); kI_lifter = i; }
      if((d != kD_lifter)) { MotorControllerPid.setD(d); kD_lifter = d; }
      if((iz != kIz)) { MotorControllerPid.setIZone(iz); kIz = iz; }
      if((ff != kFF)) { MotorControllerPid.setFF(ff); kFF = ff; }

      indexNote();
    }

    public void SetSpeed(double thisspeed) {
        Motor_Controller.set(thisspeed);
      }
    

      public void resetEncoder() {
        SetSpeed(0);
        Motor_Encoder.setPosition(Constants.DeliveryHead.Tilt_minValue);
  
        
        Motor_Controller.enableSoftLimit(SoftLimitDirection.kReverse, true);
        //enable();//reactivate the pidcontroller of this subsystem
        Motor_Encoder.setPosition(Constants.DeliveryHead.Tilt_minValue);
      }

      public void setSetpointToPosition(double position)
      {
        //enable();
        WantedEncoderValue = position;
        MotorControllerPid.setReference(WantedEncoderValue, CANSparkBase.ControlType.kPosition);
      }




      public boolean HasNote = false;


    double setpointTolerance = 2.5;
      public boolean atSetpoint() {        
          if (Constants.isWithinPercentage(CurrentEncoderValue, WantedEncoderValue, setpointTolerance)) {
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
