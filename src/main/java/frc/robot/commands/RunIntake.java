package frc.robot.commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PickupSpinner;
public class RunIntake extends Command{
    private final PickupSpinner m_PickupSpinner;
    private final Timer m_Timer = new Timer();
    public RunIntake(PickupSpinner pickupSpinner){
        m_PickupSpinner = pickupSpinner;
        addRequirements(m_PickupSpinner);
    }

    @Override
  public void initialize() {
    m_Timer.reset();
    m_Timer.start();
    
  }

  @Override
  public void execute() {
   m_PickupSpinner.RunPickup();
  }

  double PickupTimeoutSeoncds = 10.0;
  @Override
  public boolean isFinished() {
    if(m_Timer.get() > PickupTimeoutSeoncds){
        return true;
    } 
    if (m_PickupSpinner.IsNoteInPickup()) {
      m_PickupSpinner.stopSpinner();
         return true;
    }  
    return false;
  }
}
