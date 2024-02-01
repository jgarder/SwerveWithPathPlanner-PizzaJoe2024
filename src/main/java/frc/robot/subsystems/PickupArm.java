package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;

public class PickupArm extends PIDSubsystem{

    

    public double CurrentLiftEncoderValue = 0;
    public double CurrentEncoderVelocity = 0;
    public double OutputCurrent = 0;
    public double MotorTemp = 0;
    public double TempCForOverTemp = 37;

    private final CANSparkMax Motor_Controller = new CANSparkMax(Constants.PickupHead.PickUpLifterCanBusID, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
    private final RelativeEncoder Motor_Encoder = Motor_Controller.getEncoder();
    public PickupArm()
    {
        super(new PIDController(Constants.PickupHead.kP_lifter, Constants.PickupHead.kI_lifter, Constants.PickupHead.kD_lifter));//super class, must setup PID first
        //even though the default should be 0, lets tell the PID to goto 0 which is our starting position.
        Motor_Encoder.setPosition(Constants.PickupHead.minValue_Lifter);
        setSetpoint(0);
        //should the motor controller be inverted? 0 is folded in and 44 (or max) is folded out.
        Motor_Controller.setInverted(false);

        //Enable the soft limits and set the values
        Motor_Controller.enableSoftLimit(SoftLimitDirection.kForward, true);
        Motor_Controller.enableSoftLimit(SoftLimitDirection.kReverse, true);
        Motor_Controller.setSoftLimit(SoftLimitDirection.kForward, (float)Constants.PickupHead.maxValue_Lifter);
        Motor_Controller.setSoftLimit(SoftLimitDirection.kReverse, (float)Constants.PickupHead.minValue_Lifter);

        //set the idle mode to brake so it doesnt move when we dont want it to, or coast if we want it to coast after "stopping"
        Motor_Controller.setIdleMode(IdleMode.kBrake);
        
        //set the ramp rate to controll sudden input changes (smooth input
        Motor_Controller.setClosedLoopRampRate(.05);
        Motor_Controller.setOpenLoopRampRate(.05);//small ramp rate becuase this will reverse instantly. 
        
        //current limit to keep motors safe from Fire (over current)
        Motor_Controller.setSmartCurrentLimit(Constants.NeoBrushless.neo1650safelimitAmps);

        //limit everything on this motor controller to 500ms except the status 0 frame which is 10ms and does faults and applied output. 
        Motor_Controller.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);  //Default Rate: 20ms ,Motor Velocity,Motor Temperature,Motor VoltageMotor Current
        Motor_Controller.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);  //Default Rate: 20ms ,Motor Position
        Motor_Controller.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500); //Default Rate: 50ms ,Analog Sensor Voltage ,Analog Sensor Velocity ,Analog Sensor Position
        Motor_Controller.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 500); //Default Rate: 20ms, Alternate Encoder Velocity,Alternate Encoder Position
        Motor_Controller.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 500); //Default Rate: 200ms, Duty Cycle Absolute Encoder Position,Duty Cycle Absolute Encoder Absolute Angle
        Motor_Controller.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 500); //Default Rate: 200ms, Duty Cycle Absolute Encoder Velocity,Duty Cycle Absolute Encoder Frequency
        
        
        enable();//enable the pidcontroller of this subsystem 


    }

    @Override
    public double getMeasurement() {
        //INSIDE GET ENCODER DATA WE UPDATE OUR CurrentLiftEncoderValue! this is how the PID WORKS!!!!
        getEncoderData();
        
        return CurrentLiftEncoderValue;
    }

    @Override
    public void useOutput(double output, double setpoint) {
      SetSpeed(output);
      SmartDashboard.putNumber("Motor_Encoder 1 PID output",output);
      SmartDashboard.putNumber("Motor_Encoder 1 SetPoint",setpoint);
      //m_shooterMotor.setVoltage(output + m_shooterFeedforward.calculate(setpoint));
    }

    public void getEncoderData()
    {
      OutputCurrent = Motor_Controller.getOutputCurrent();
      SmartDashboard.putNumber("Motor Controller 1 Amps",OutputCurrent);
  
      MotorTemp = Motor_Controller.getMotorTemperature();
      SmartDashboard.putNumber("Motor Controller 1 Motor Temp",MotorTemp);
      /**
       * Encoder position is read from a RelativeEncoder object by calling the
       * GetPosition() method.
       * 
       * GetPosition() returns the position of the encoder in units of revolutions
       */
      CurrentLiftEncoderValue = Motor_Encoder.getPosition();
      SmartDashboard.putNumber("Motor_Encoder 1 PID Encoder Position",CurrentLiftEncoderValue);
  
      /**
       * Encoder velocity is read from a RelativeEncoder object by calling the
       * GetVelocity() method.
       * 
       * GetVelocity() returns the velocity of the encoder in units of RPM
       */
      CurrentEncoderVelocity = Motor_Encoder.getVelocity();
      SmartDashboard.putNumber("Motor_Encoder 1 Velocity", CurrentEncoderVelocity);
  
    }

    @Override
    public void periodic() {
        //anything you wanted to do periodically put it here.

      super.periodic();// This is a PidSubsystem, we have orridden the periodic method to get encoder data... So we need to call the super periodic method to get the PID stuff to work.
    }

    public void SetSpeed(double thisspeed) {
        Motor_Controller.set(thisspeed);
    }
    
      private void retractSlowly() {
        int reduction = 10;
        if(CurrentLiftEncoderValue > reduction)
        {
          setSetpoint(CurrentLiftEncoderValue-reduction);
        }
      }
      ///rewind and reset beyond softlimit to reset head position.
      SlewRateLimiter speedLimiter = new SlewRateLimiter(.5);
      public void slowWindInBeyondSoftLimit() {
        WindInBeyondSoftLimit(-.6);
      }
      public void slowerWindInBeyondSoftLimit() {
        WindInBeyondSoftLimit(-.1);
      }
      public void WindInBeyondSoftLimit(double retractSpeed) {
        disable(); //disable the pidcontroller of this subsystem
        Motor_Controller.enableSoftLimit(SoftLimitDirection.kReverse, false);
        SetSpeed(speedLimiter.calculate(retractSpeed));
      }
      public void resetEncoder() {
        SetSpeed(0);
        setSetpointZero();
        
        Motor_Controller.enableSoftLimit(SoftLimitDirection.kReverse, true);
        enable();//reactivate the pidcontroller of this subsystem
        Motor_Encoder.setPosition(Constants.PickupHead.minValue_Lifter);
      }

      public void setSetpointZero() {
        enable();
        setSetpoint(Constants.PickupHead.minValue_Lifter);
        pickupState = pickupState.ZERO;
      }
    public void setSetpointVerticle() {
        enable();
        setSetpoint(Constants.PickupHead.PickupVertical);
        pickupState = pickupState.Vertical;
    }
    public void setSetpointFloorPickup() {
        enable();
        setSetpoint(Constants.PickupHead.PickupFloorPickup);
        pickupState = pickupState.FloorPickup;
    }
    public void setSetpointSourcePickup() {
        enable();
        setSetpoint(Constants.PickupHead.PickupSourcePickup);
        pickupState = pickupState.SourcePickup;
    }
        public void setSetpointPassingPickup() {
        enable();
        setSetpoint(Constants.PickupHead.PickupPassing);
        pickupState = pickupState.passToHead;
    }
    double windback = 3.0;
    public void HoldAutoLoaded(){
        disable();
        pickupState = pickupState.ZERO;
        double position = Motor_Encoder.getPosition();
        Motor_Encoder.setPosition(position);
        setSetpoint(position-windback);
        enable();
      }


      PickupState pickupState = PickupState.Startup;
      public boolean HasNote = false;
      public enum PickupState
      {
        Startup,
        ZERO,
        Vertical,
        SourcePickup,
        FloorPickup,
        passToHead,
        SpeakerReady,
        AmpReady,
        TrapReady,
      }
       double setpointTolerance = 2.0;
      public boolean atSetpoint() {
        if (m_enabled) {

          //return m_controller.atSetpoint();
          double setpointGoal = getSetpoint();
         
          if (Constants.isWithinPercentage(CurrentLiftEncoderValue, setpointGoal, setpointTolerance)) {
            return true;
          } else {
            return false; 
          }
        } else {
          return false; 
        }
      }
}
