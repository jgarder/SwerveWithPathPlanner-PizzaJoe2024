package frc.robot.commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DeliveryShooter;
import frc.robot.subsystems.PickupSpinner;
public class SpoolPizzaDeliveryToRPM extends Command{
    private final DeliveryShooter m_DeliveryShooter;
    private final Timer m_Timer = new Timer();
    public SpoolPizzaDeliveryToRPM(DeliveryShooter pickupSpinner){
        m_DeliveryShooter = pickupSpinner;
        addRequirements(m_DeliveryShooter);
    }

    @Override
  public void initialize() {
    m_Timer.reset();
    m_Timer.start();
    m_DeliveryShooter.SetShootSpeed(Constants.DeliveryHead.ShooterRpmSpeakerClose);
  }

  @Override
  public void execute() {
   //m_DeliveryShooter.RunPickup();
  }

  double PickupTimeoutSeoncds = 10.0;
  @Override
  public boolean isFinished() {
    if(m_Timer.get() > PickupTimeoutSeoncds){
        return true;
    } 
    double percentageTolerance = 10;
    boolean isUpperWithinRange = Constants.isWithinPercentage(m_DeliveryShooter.CurrentEncoderVelocity, m_DeliveryShooter.LastSetRPM, percentageTolerance);
    boolean islowerWithinRange = Constants.isWithinPercentage(m_DeliveryShooter.CurrentEncoderValue_LowS, m_DeliveryShooter.LastSetRPM, percentageTolerance);
    if (islowerWithinRange & isUpperWithinRange) {
         return true;
    }  
    return false;
  }
}
