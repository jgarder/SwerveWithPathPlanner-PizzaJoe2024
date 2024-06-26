package frc.robot.commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer.PizzaManager;
import frc.robot.subsystems.DeliveryTilt;

public class ZeroDtilt extends Command{
    //private final PickupSpinner m_PickupSpinner;
    private final DeliveryTilt m_DeliveryTilter;
    private final Timer m_Timer = new Timer();
    //double desiredSetpoint = 0;
    public ZeroDtilt(DeliveryTilt DelivTilt)
    {
        //m_PickupSpinner = pickupSpinner;
        m_DeliveryTilter = DelivTilt;
        //this.desiredSetpoint = DesiredSetpoint;
        //addRequirements(m_PickupSpinner);
        addRequirements(m_DeliveryTilter);
    }

    @Override
  public void initialize() {
    m_Timer.reset();
    m_Timer.start();
    if(PizzaManager.HasTiltBeenZeroed){end(false);return;}
    m_DeliveryTilter.setSetpointToPosition(0);
  }

  @Override
  public void execute() {
   
  }

      // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
  }

  double TimeoutSeconds = .05;
  @Override
  public boolean isFinished() {
    if(m_Timer.get() > TimeoutSeconds){
      
      if(PizzaManager.HasTiltBeenZeroed)
      {
        Constants.DeliveryHead.Tilt_Position_Speaker_Closest=  Constants.DeliveryHead.Tilt_Position_Speaker_Closest - Constants.DeliveryHead.offset;
        Constants.DeliveryHead.Tilt_Position_Speaker_Mid =  Constants.DeliveryHead.Tilt_Position_Speaker_Mid  - Constants.DeliveryHead.offset;
        Constants.DeliveryHead.Tilt_Position_Speaker_Furthest = Constants.DeliveryHead.Tilt_Position_Speaker_Furthest  - Constants.DeliveryHead.offset;
      }
      Constants.DeliveryHead.offset = m_DeliveryTilter.Motor_Controller.getPosition().getValueAsDouble();
      Constants.DeliveryHead.Tilt_Position_Speaker_Closest=  Constants.DeliveryHead.Tilt_Position_Speaker_Closest + Constants.DeliveryHead.offset;
        Constants.DeliveryHead.Tilt_Position_Speaker_Mid =  Constants.DeliveryHead.Tilt_Position_Speaker_Mid  + Constants.DeliveryHead.offset;
        Constants.DeliveryHead.Tilt_Position_Speaker_Furthest = Constants.DeliveryHead.Tilt_Position_Speaker_Furthest  + Constants.DeliveryHead.offset;
       SmartDashboard.putNumber(m_DeliveryTilter.MotorName + " Closest Setpoint", Constants.DeliveryHead.Tilt_Position_Speaker_Closest);
        SmartDashboard.putNumber(m_DeliveryTilter.MotorName + " Mid Setpoint", Constants.DeliveryHead.Tilt_Position_Speaker_Mid);
        SmartDashboard.putNumber(m_DeliveryTilter.MotorName + " Furthest Setpoint", Constants.DeliveryHead.Tilt_Position_Speaker_Furthest);
      //m_DeliveryTilter.Motor_Controller.setPosition(0);
      //m_DeliveryTilter.disableatpark();
      PizzaManager.HasTiltBeenZeroed = true;
      
      //m_DeliveryTilter.setSetpointToPosition(0);
        return true;
    } 
    //  if (m_DeliveryTilter.Motor_Controller.getPosition().getValueAsDouble() < -1.0) {
    //       m_DeliveryTilter.Motor_Controller.setPosition(0);
    //       m_DeliveryTilter.setSetpointToPosition(0);
    //     return true;
     //}  
    return false;
  }
}
