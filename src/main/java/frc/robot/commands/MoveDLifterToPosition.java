package frc.robot.commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DeliveryLifter;
import frc.robot.subsystems.PickupArm;
import frc.robot.subsystems.PickupSpinner;

public class MoveDLifterToPosition extends Command{
    //private final PickupSpinner m_PickupSpinner;
    private final DeliveryLifter m_DeliveryLifter;
    private final Timer m_Timer = new Timer();
    double desiredSetpoint = 0;
    public MoveDLifterToPosition(double DesiredSetpoint, DeliveryLifter DelivLifter)
    {
        //m_PickupSpinner = pickupSpinner;
        m_DeliveryLifter = DelivLifter;
        this.desiredSetpoint = DesiredSetpoint;
        //addRequirements(m_PickupSpinner);
        addRequirements(m_DeliveryLifter);
    }

    @Override
  public void initialize() {
    m_Timer.reset();
    m_Timer.start();
    m_DeliveryLifter.setSetpoint(desiredSetpoint);
  }

  @Override
  public void execute() {
   
  }

  double TimeoutSeconds = 10.0;
  @Override
  public boolean isFinished() {
    if(m_Timer.get() > TimeoutSeconds){
        return true;
    } 
    if (m_DeliveryLifter.atSetpoint()) {
         return true;
    }  
    return false;
  }
}
