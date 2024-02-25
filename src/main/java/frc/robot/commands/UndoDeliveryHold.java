package frc.robot.commands;
import com.revrobotics.CANSparkBase;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer.PizzaManager;
import frc.robot.subsystems.DeliveryHolder;
public class UndoDeliveryHold extends Command{
    private final DeliveryHolder m_DeliveryHolder;
    private final Timer m_Timer = new Timer();
    //private final boolean IgnorelimitSwitch;
    private double reduction = -20;
    double Timeout = 10;//.5
    public UndoDeliveryHold(DeliveryHolder deliveryHolder,double timeout){
        m_DeliveryHolder = deliveryHolder;
        Timeout = timeout;
        //IgnorelimitSwitch = ignoreLimitSwitch;
        //reduction = speed;
        addRequirements(m_DeliveryHolder);
    }

    @Override
  public void initialize() {
    //PizzaManager.NoteInDeliveryHolder = false;
    m_Timer.reset();
    m_Timer.start();
    m_DeliveryHolder.m_forwardLimit.enableLimitSwitch(false);
    m_DeliveryHolder.SetToWantedDutyCycle(-1);
    System.out.println("executing Undo");
  }
  
  @Override
  public void execute() {
    //System.out.println("executing Undo");
    //m_DeliveryHolder.SetToWantedDutyCycle(1);
   //m_DeliveryHolder.MovePosition(reduction);
  }

  
  @Override
  public boolean isFinished() {
    if(m_Timer.get() > Timeout){
      m_DeliveryHolder.m_forwardLimit.enableLimitSwitch(true);
      m_DeliveryHolder.stopSpinner();
        return true;
    } 
    if (!m_DeliveryHolder.IsNoteInDeliveryHold() ) {
      m_DeliveryHolder.m_forwardLimit.enableLimitSwitch(true);
      m_DeliveryHolder.stopSpinner();
          return true;
    }  
    return false;
  }
}
