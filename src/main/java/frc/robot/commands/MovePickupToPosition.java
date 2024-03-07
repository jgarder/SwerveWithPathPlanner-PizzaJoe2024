package frc.robot.commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PickupArm;
import frc.robot.subsystems.PickupSpinner;

public class MovePickupToPosition extends Command{
    //private final PickupSpinner m_PickupSpinner;
    private final PickupArm m_pickuparm;
    private final Timer m_Timer = new Timer();
    double desiredSetpoint = 0;
    public MovePickupToPosition(double DesiredSetpoint, PickupArm pickuparm)
    {
        //m_PickupSpinner = pickupSpinner;
        m_pickuparm = pickuparm;
        this.desiredSetpoint = DesiredSetpoint;
        //addRequirements(m_PickupSpinner);
        addRequirements(m_pickuparm);
    }

    @Override
  public void initialize() {
    m_Timer.reset();
    m_Timer.start();
    m_pickuparm.setSetpointToPosition(desiredSetpoint);
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
    if (m_pickuparm.atSetpoint()) {
         return true;
    }  
    return false;
  }
}
