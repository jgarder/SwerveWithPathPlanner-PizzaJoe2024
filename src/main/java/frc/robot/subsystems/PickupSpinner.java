package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;

import frc.robot.Constants;

public class PickupSpinner extends PIDSubsystem{

    public String MotorName = "PickUp Intake";
    public double CurrentEncoderValue = 0;
    public double CurrentEncoderVelocity = 0;
    public double OutputCurrent = 0;
    public double MotorTemp = 0;
    public double TempCForOverTemp = 37;

    boolean NoteInPickup = false;

    private final CANSparkMax Motor_Controller = new CANSparkMax(Constants.PickupHead.PickUpSpinnerCanBusID, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
    private final RelativeEncoder Motor_Encoder = Motor_Controller.getEncoder();
    public SparkLimitSwitch m_forwardLimit = Motor_Controller.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
    public PickupSpinner()
    {
        super(new PIDController(Constants.PickupHead.kP_Spinner, Constants.PickupHead.kI_Spinner, Constants.PickupHead.kD_Spinner));//super class, must setup PID first
         //even though the default should be 0, lets tell the PID to goto 0 which is our starting position.
         Motor_Encoder.setPosition(Constants.PickupHead.minValue_Spinner);
         setSetpoint(0);

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
        
        return CurrentEncoderValue;
    }

    @Override
    public void useOutput(double output, double setpoint) {
      SetSpeed(output);
      SmartDashboard.putNumber(MotorName + " PID output",output);
      SmartDashboard.putNumber(MotorName + " SetPoint",setpoint);
      //m_shooterMotor.setVoltage(output + m_shooterFeedforward.calculate(setpoint));
    }
    
    public boolean setIsnoteInPickup(boolean isnoteinpickup)
    {
      NoteInPickup = isnoteinpickup;
      return NoteInPickup;
    }
    public boolean IsNoteInPickup()
    {
      return NoteInPickup;
    }
    public void SetSpeed(double thisspeed) {
        Motor_Controller.set(thisspeed);
    }

    public void RunPickup() {
        int reduction = 15;
        //if our limit switch is turned on And there is no note in the pickup, then we can run the pickup motor.. 
        if (SmartDashboard.getBoolean(MotorName + " Forward Limit Enabled", true)) 
        {
            boolean istriggered = m_forwardLimit.isPressed();
            if (istriggered) {//we have a note in pickup and limit switch is pressed, so we need to reduce the setpoint to hold the note in pickup.
                if (!NoteInPickup) {
                    HoldAutoLoaded();
                NoteInPickup = true;
                }
                else
                {
                    NoteInPickup = true;
                    //trying to run pickup but the limit switch is enabled and there is a note in the pickup so dont run the head. 
                }
                return;   
            }   
      
        }
        setSetpoint(CurrentEncoderValue-reduction);
    }
    //
    double windback = 3;
    public void HoldAutoLoaded(){
        disable();
        //pickupState = pickupState.ZERO;
        double position = Motor_Encoder.getPosition();
        Motor_Encoder.setPosition(position);
        setSetpoint(position-windback);
    
        enable();
    }
      double releaseDistance = 30;
    public void ReleaseNote(){
        disable();
        //pickupState = pickupState.ZERO;
        double position = Motor_Encoder.getPosition();
        Motor_Encoder.setPosition(position);
        setSetpoint(position+releaseDistance);
        enable();
        setIsnoteInPickup(false);
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
        
        SmartDashboard.putBoolean(MotorName + " Forward Limit Enabled", m_forwardLimit.isLimitSwitchEnabled());
        SmartDashboard.putBoolean(MotorName + " Forward Limit Triggered", m_forwardLimit.isPressed());
      }
    @Override
    public void periodic() {
         // enable/disable limit switches based on value read from SmartDashboard
    m_forwardLimit.enableLimitSwitch(SmartDashboard.getBoolean(MotorName + " Forward Limit Enabled", true));
    super.periodic();// This is a PidSubsystem, we have orridden the periodic method to get encoder data... So we need to call the super periodic method to get the PID stuff to work.


  }
}
