package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PickupArm extends SubsystemBase{

    

    private static final String MotorName = "PickupArm";
    public double CurrentLiftEncoderValue = 0;
    public double CurrentEncoderVelocity = 0;
    public double OutputCurrent = 0;
    public double MotorTemp = 0;
    public double TempCForOverTemp = 37;
    double kMaxOutput = 1; 
    double kMinOutput = -1;
    double kP_lifter = Constants.PizzaFloorPickupHead.kP_FloorPickupArm;
    double kI_lifter = Constants.PizzaFloorPickupHead.kI_FloorPickupArm;
    double kD_lifter = Constants.PizzaFloorPickupHead.kD_FloorPickupArm;
    double setpointGoal = 0;
    
    private final CANSparkMax Motor_Controller = new CANSparkMax(Constants.CANBus.PickUpLifterCanBusID, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
    private final RelativeEncoder Motor_Encoder = Motor_Controller.getEncoder();
     private final SparkPIDController MotorControllerPid = Motor_Controller.getPIDController();
    public PickupArm()
    {
        //even though the default should be 0, lets tell the PID to goto 0 which is our starting position.
        Motor_Encoder.setPosition(Constants.PizzaFloorPickupHead.minValue_PickupArm);
        //should the motor controller be inverted? 0 is folded in and 44 (or max) is folded out.
        Motor_Controller.setInverted(false);

        //Enable the soft limits and set the values
        Motor_Controller.enableSoftLimit(SoftLimitDirection.kForward, true);
        Motor_Controller.enableSoftLimit(SoftLimitDirection.kReverse, true);
        Motor_Controller.setSoftLimit(SoftLimitDirection.kForward, (float)Constants.PizzaFloorPickupHead.maxValue_PickupArm);
        Motor_Controller.setSoftLimit(SoftLimitDirection.kReverse, (float)Constants.PizzaFloorPickupHead.minValue_PickupArm);

        //set the idle mode to brake so it doesnt move when we dont want it to, or coast if we want it to coast after "stopping"
        Motor_Controller.setIdleMode(IdleMode.kBrake);
        
        //set the ramp rate to controll sudden input changes (smooth input
        Motor_Controller.setClosedLoopRampRate(.05);
        Motor_Controller.setOpenLoopRampRate(.05);//small ramp rate becuase this will reverse instantly. 
        
        //current limit to keep motors safe from Fire (over current)
        Motor_Controller.setSmartCurrentLimit(20);


        MotorControllerPid.setP(Constants.PizzaFloorPickupHead.kP_FloorPickupArm);
        MotorControllerPid.setI(Constants.PizzaFloorPickupHead.kI_FloorPickupArm);
        MotorControllerPid.setD(Constants.PizzaFloorPickupHead.kD_FloorPickupArm);
        //MotorControllerPid.setFF(kFF);
        MotorControllerPid.setOutputRange(kMinOutput, kMaxOutput);
        MotorControllerPid.setReference(0.0, CANSparkBase.ControlType.kPosition);

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

        

        // display PID coefficients on SmartDashboard
        SmartDashboard.putNumber(MotorName + " P Gain", Constants.PizzaFloorPickupHead.kP_FloorPickupArm);
        SmartDashboard.putNumber(MotorName + " I Gain", Constants.PizzaFloorPickupHead.kI_FloorPickupArm);
        SmartDashboard.putNumber(MotorName + " D Gain", Constants.PizzaFloorPickupHead.kD_FloorPickupArm);
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
  
    }

    @Override
    public void periodic() {
      getEncoderData();
        //anything you wanted to do periodically put it here.
      SmartDashboard.putNumber(MotorName + " SetPoint",setpointGoal);
      // read PID coefficients from SmartDashboard
      double p = SmartDashboard.getNumber(MotorName + " P Gain", 0);
      double i = SmartDashboard.getNumber(MotorName + " I Gain", 0);
      double d = SmartDashboard.getNumber(MotorName + " D Gain", 0);
        
      if((p != kP_lifter)) { MotorControllerPid.setP(p); kP_lifter = p;  }
      if((i != kI_lifter)) { MotorControllerPid.setI(i); kI_lifter = i; }
      if((d != kD_lifter)) { MotorControllerPid.setD(d); kD_lifter = d; }
      }

    
  
      public void setSetpointToPosition(double ToPosition) {
        setpointGoal = ToPosition;
        MotorControllerPid.setReference(setpointGoal, CANSparkBase.ControlType.kPosition);
      }
      // public void setSetpointZero() {
      //   setpointGoal = Constants.PizzaFloorPickupHead.minValue_PickupArm;
      //   MotorControllerPid.setReference(setpointGoal, CANSparkBase.ControlType.kPosition);
      // }
      // public void setSetpointVerticle() {
      //   setpointGoal = Constants.PizzaFloorPickupHead.PickupVertical;
      //   MotorControllerPid.setReference(setpointGoal, CANSparkBase.ControlType.kPosition);
      // }
      // public void setSetpointFloorPickup() {
      //   setpointGoal = Constants.PizzaFloorPickupHead.PickupFloorPickup;
      //   MotorControllerPid.setReference(setpointGoal, CANSparkBase.ControlType.kPosition);
      // }
      // public void setSetpointSourcePickup() {
      //   setpointGoal = Constants.PizzaFloorPickupHead.PickupSourcePickup;
      //   MotorControllerPid.setReference(setpointGoal, CANSparkBase.ControlType.kPosition);
      // }
      // public void setSetpointPassingPickup() {
      //   setpointGoal = Constants.PizzaFloorPickupHead.PickupPassing;
      //   MotorControllerPid.setReference(setpointGoal, CANSparkBase.ControlType.kPosition);
      // }

    // double windback = 3.0;
    // public void HoldAutoLoaded(){
    //     disable();
    //     double position = Motor_Encoder.getPosition();
    //     Motor_Encoder.setPosition(position);
    //     setSetpoint(position-windback);
    //     enable();
    //   }


      double setpointTolerance = 2.0;
      public boolean atSetpoint() {
          if (Constants.isWithinAmount(CurrentLiftEncoderValue, setpointGoal, setpointTolerance)) {
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
