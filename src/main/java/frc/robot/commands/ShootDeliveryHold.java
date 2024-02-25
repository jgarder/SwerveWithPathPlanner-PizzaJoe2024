package frc.robot.commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer.PizzaManager;
import frc.robot.subsystems.DeliveryHolder;
public class ShootDeliveryHold extends Command{
    private final DeliveryHolder m_DeliveryHolder;
    private final Timer m_Timer = new Timer();
    private double reduction = 40;
    
    public ShootDeliveryHold(DeliveryHolder deliveryHolder){
        m_DeliveryHolder = deliveryHolder;
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
   m_DeliveryHolder.IntakeRuntoHoldCommand(reduction,true);
  }

  double TimeoutSeconds = .3;
  @Override
  public boolean isFinished() {
    if(m_Timer.get() > TimeoutSeconds){
      m_DeliveryHolder.MovePosition(0);
        return true;
    } 
    // if (!m_DeliveryHolder.IsNoteInDeliveryHold()) {
    //   //m_DeliveryHolder.MovePosition(20);//This is a post running sequence that need not be run at this moment. add it back if needed?
    //       return true;
    // }  
    return false;
  }
}
