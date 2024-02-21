package frc.robot.commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ChainLifterS;
import frc.robot.subsystems.PickupArm;
import frc.robot.subsystems.PickupSpinner;

public class MoveChainLiftToPosition extends Command{
    //private final PickupSpinner m_PickupSpinner;
    private final ChainLifterS m_ChainLifterS;
    private final Timer m_Timer = new Timer();
    double desiredSetpoint = 0;
    public MoveChainLiftToPosition(double DesiredSetpoint, ChainLifterS ChainLifterS)
    {
        //m_PickupSpinner = pickupSpinner;
        m_ChainLifterS = ChainLifterS;
        this.desiredSetpoint = DesiredSetpoint;
        //addRequirements(m_PickupSpinner);
        addRequirements(m_ChainLifterS);
    }

    @Override
  public void initialize() {
    m_Timer.reset();
    m_Timer.start();
    //m_ChainLifterS.setSetpoint(desiredSetpoint);
    //m_ChainLifterS.enable();
    m_ChainLifterS.setSetpointToPosition(desiredSetpoint);
  }

  @Override
  public void execute() {
   
  }

  double TimeoutSeconds = 60.0;
  @Override
  public boolean isFinished() {
    if(m_Timer.get() > TimeoutSeconds){
        return true;
    } 
    if (m_ChainLifterS.atSetpoint()) {
         return true;
    }  
    return false;
  }
}
