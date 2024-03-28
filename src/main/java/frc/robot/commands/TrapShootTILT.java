package frc.robot.commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DeliveryTilt;

public class TrapShootTILT extends Command{
    //private final PickupSpinner m_PickupSpinner;
    private final DeliveryTilt m_DeliveryTilter;
    private final Timer m_Timer = new Timer();
    double desiredSetpoint = 0;
    public TrapShootTILT(DeliveryTilt DelivTilt)
    {
        //m_PickupSpinner = pickupSpinner;
        m_DeliveryTilter = DelivTilt;
        //addRequirements(m_PickupSpinner);
        addRequirements(m_DeliveryTilter);
    }
    @Override
  public void initialize() {
    m_Timer.reset();
    m_Timer.start();
    m_SettleTimer.reset();
    m_SettleTimer.start();
    this.desiredSetpoint = SmartDashboard.getNumber("TrapFloorTilt",0);
    m_DeliveryTilter.setSetpointToPosition(desiredSetpoint);
  }

  @Override
  public void execute() {
   
  }
  double SettleTimeAtCorrectPosition = .15;
  double TimeoutSeconds = 10.0;
  private final Timer m_SettleTimer = new Timer();
  boolean ReadyTofire = false;
  @Override
  public boolean isFinished() {
    if(m_Timer.get() > TimeoutSeconds){
        return true;
    } 
    if (m_DeliveryTilter.atSetpoint(Constants.DeliveryHead.TiltsetpointTolerance,Constants.DeliveryHead.TiltSettleTimeAtPosition)) {
      //if(m_SettleTimer.get() > SettleTimeAtCorrectPosition)
      //{
        ReadyTofire = true;
        return true;
      //}
      
         
    } 
    else
      {
        m_SettleTimer.reset();
        m_SettleTimer.start();
      }
    return false;
  }
}
