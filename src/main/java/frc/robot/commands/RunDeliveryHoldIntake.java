package frc.robot.commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer.PizzaManager;
import frc.robot.subsystems.DeliveryHolder;
public class RunDeliveryHoldIntake extends Command{
    private final DeliveryHolder m_DeliveryHolder;
    private final Timer m_Timer = new Timer();
    private final boolean IgnorelimitSwitch;
    private double reduction = 40;
    public RunDeliveryHoldIntake(DeliveryHolder deliveryHolder,boolean ignoreLimitSwitch,double speed){
        m_DeliveryHolder = deliveryHolder;
        IgnorelimitSwitch = ignoreLimitSwitch;
        reduction = speed;
        addRequirements(m_DeliveryHolder);
    }

    @Override
  public void initialize() {
    PizzaManager.NoteInDeliveryHolder = false;
    m_Timer.reset();
    m_Timer.start();
    
  }
  
  @Override
  public void execute() {
    //System.out.println("executing hold");
   m_DeliveryHolder.IntakeRuntoHoldCommand(reduction,IgnorelimitSwitch);
  }

  double PickupTimeoutSeoncds = 5.0;
  @Override
  public boolean isFinished() {
    if(m_Timer.get() > PickupTimeoutSeoncds){
        return true;
    } 
    if (m_DeliveryHolder.IsNoteInDeliveryHold() & !IgnorelimitSwitch) {
      //m_DeliveryHolder.MovePosition(20);//This is a post running sequence that need not be run at this moment. add it back if needed?
          return true;
    }  
    return false;
  }
}
