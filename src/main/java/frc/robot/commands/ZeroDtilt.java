package frc.robot.commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
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
    m_DeliveryTilter.setSetpointToPosition(-5);
  }

  @Override
  public void execute() {
   
  }

  double TimeoutSeconds = .5;
  @Override
  public boolean isFinished() {
    if(m_Timer.get() > TimeoutSeconds){
      m_DeliveryTilter.Motor_Controller.setPosition(0);
      m_DeliveryTilter.setSetpointToPosition(0);
      m_DeliveryTilter.disableatpark();
      PizzaManager.HasTiltBeenZeroed = true;
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
