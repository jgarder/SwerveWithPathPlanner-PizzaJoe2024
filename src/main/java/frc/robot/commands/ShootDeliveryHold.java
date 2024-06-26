package frc.robot.commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer.PizzaManager;
import frc.robot.subsystems.DeliveryHolder;
public class ShootDeliveryHold extends Command{
    private final DeliveryHolder m_DeliveryHolder;
    private final Timer m_Timer = new Timer();
    //private double reduction = 40;
    
    public ShootDeliveryHold(DeliveryHolder deliveryHolder){
        m_DeliveryHolder = deliveryHolder;
        addRequirements(m_DeliveryHolder);
    }
     public ShootDeliveryHold(DeliveryHolder deliveryHolder,double ForSeconds,double mintimeout){
      TimeoutSeconds=ForSeconds;
      MinTimeoutSeconds = mintimeout;
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
   //m_DeliveryHolder.IntakeRuntoHoldCommand(reduction,true);
   m_DeliveryHolder.m_forwardLimit.enableLimitSwitch(false);
   m_DeliveryHolder.SetToWantedDutyCycle(m_DeliveryHolder.SpeakerDutyCycle);
  }

  double TimeoutSeconds = 1;
   double MinTimeoutSeconds = .15;//.125;//.15;
  @Override
  public boolean isFinished() {
    if(m_Timer.get() > TimeoutSeconds){
      //m_DeliveryHolder.MovePosition(0);
      m_DeliveryHolder.SetToWantedDutyCycle(0);
       m_DeliveryHolder.m_forwardLimit.enableLimitSwitch(true);
        return true;
    } 
    if (m_Timer.get() > MinTimeoutSeconds && !m_DeliveryHolder.IsNoteInDeliveryHold()) {
        m_DeliveryHolder.SetToWantedDutyCycle(0);
       m_DeliveryHolder.m_forwardLimit.enableLimitSwitch(true);
           return true;
     }  
    return false;
  }
}
