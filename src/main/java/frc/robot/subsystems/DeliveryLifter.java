package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import edu.wpi.first.hal.CANAPITypes.CANDeviceType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DeliveryLifter extends SubsystemBase {

    
  private static final String MotorName = "DeliveryLift";
    public double CurrentLiftEncoderValue = 0;
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
    private final CANSparkMax Motor_Controller = new CANSparkMax(Constants.CANBus.Lift_CanBusID, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
    private final RelativeEncoder Motor_Encoder = Motor_Controller.getEncoder();
    private final SparkPIDController MotorControllerPid = Motor_Controller.getPIDController();
    public DeliveryLifter()
    {
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
        Motor_Controller.setInverted(false);

        //Enable the soft limits and set the values
        Motor_Controller.enableSoftLimit(SoftLimitDirection.kForward, true);
        Motor_Controller.enableSoftLimit(SoftLimitDirection.kReverse, true);
        Motor_Controller.setSoftLimit(SoftLimitDirection.kForward, (float)Constants.DeliveryHead.Lift_maxValue);
        Motor_Controller.setSoftLimit(SoftLimitDirection.kReverse, (float)Constants.DeliveryHead.Lift_minValue);

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
        Motor_Controller.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 250); //Default Rate: 50ms ,Analog Sensor Voltage ,Analog Sensor Velocity ,Analog Sensor Position
        Motor_Controller.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 250); //Default Rate: 20ms, Alternate Encoder Velocity,Alternate Encoder Position
        Motor_Controller.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 500); //Default Rate: 200ms, Duty Cycle Absolute Encoder Position,Duty Cycle Absolute Encoder Absolute Angle
        Motor_Controller.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 500); //Default Rate: 200ms, Duty Cycle Absolute Encoder Velocity,Duty Cycle Absolute Encoder Frequency
           
        
        // display PID coefficients on SmartDashboard
        SmartDashboard.putNumber(MotorName + " P Gain", kP_lifter);
        SmartDashboard.putNumber(MotorName + " I Gain", kI_lifter);
        SmartDashboard.putNumber(MotorName + " D Gain", kD_lifter);
        SmartDashboard.putNumber(MotorName + " I Zone", kIz);
        SmartDashboard.putNumber(MotorName + " Feed Forward", kFF);

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
      CurrentLiftEncoderValue = Motor_Encoder.getPosition();
      SmartDashboard.putNumber(MotorName + " PID Encoder Position",CurrentLiftEncoderValue);
  
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
    
    }

    public void SetSpeed(double thisspeed) {
        Motor_Controller.set(thisspeed);
      }
    

      public void resetEncoder() {
        SetSpeed(0);
        Motor_Encoder.setPosition(Constants.PickupHead.minValue_PickupArm);
        setSetpointZero();
        
        Motor_Controller.enableSoftLimit(SoftLimitDirection.kReverse, true);
        //enable();//reactivate the pidcontroller of this subsystem
        Motor_Encoder.setPosition(Constants.PickupHead.minValue_PickupArm);
      }

      public void setSetpoint(double setpoint) {
        //enable();
        WantedEncoderValue = setpoint;
        MotorControllerPid.setReference(WantedEncoderValue, CANSparkBase.ControlType.kPosition);
      }

      public void setSetpointZero() {
        //enable();
        WantedEncoderValue = Constants.DeliveryHead.Lift_Position_Zero;
        MotorControllerPid.setReference(WantedEncoderValue, CANSparkBase.ControlType.kPosition);
      }
      public void setSetpointPassing() {
        //enable();
        WantedEncoderValue = Constants.DeliveryHead.Lift_Position_Passing;
        MotorControllerPid.setReference(WantedEncoderValue, CANSparkBase.ControlType.kPosition);
      }
      public void setSetpointAmp() {
        //enable();
        WantedEncoderValue = Constants.DeliveryHead.Lift_Position_Amp;
        MotorControllerPid.setReference(WantedEncoderValue, CANSparkBase.ControlType.kPosition);
      }
      public void setSetpointTrap() {
        //enable();
        WantedEncoderValue = Constants.DeliveryHead.Lift_Position_TrapStart;
        MotorControllerPid.setReference(WantedEncoderValue, CANSparkBase.ControlType.kPosition);
      }
      


      public boolean HasNote = false;
       double setpointTolerance = 7;
      public boolean atSetpoint() {
        if(Constants.isWithinAmount(WantedEncoderValue, CurrentLiftEncoderValue, setpointTolerance)){
          return true;
        }
        return false;
      }
}
