package frc.robot.commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer.PizzaManager;
import frc.robot.RobotContainer.PizzaManager.PizzaTracker;
import frc.robot.subsystems.DeliveryHolder;
import frc.robot.subsystems.PickupSpinner;
public class WaitForIndexCMD extends Command{
    private final DeliveryHolder m_DeliveryHolder;
    private final Timer m_Timer = new Timer();
    public WaitForIndexCMD(DeliveryHolder deliveryholder){
        m_DeliveryHolder = deliveryholder;
        //addRequirements(m_DeliveryHolder);
    }

    @Override
  public void initialize() {
    m_Timer.reset();
    m_Timer.start();
    
  }

  @Override
  public void execute() {
  }

  double PickupTimeoutSeoncds = 4;
  //double forcedruntime = .25;
  @Override
  public boolean isFinished() {
    // if(m_Timer.get() < forcedruntime){
    //     return false;
    // }  
    if(m_Timer.get() > PickupTimeoutSeoncds){
        return true;
    } 
    if (PizzaManager.pizzaStage == PizzaTracker.Indexed || m_DeliveryHolder.requestingIndex == false) {
         return true;
    }  
    return false;
  }
}
