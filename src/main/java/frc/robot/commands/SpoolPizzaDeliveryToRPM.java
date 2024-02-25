package frc.robot.commands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DeliveryShooter;
import frc.robot.subsystems.PickupSpinner;
public class SpoolPizzaDeliveryToRPM extends Command{
    private final DeliveryShooter m_DeliveryShooter;
    private final Timer m_Timer = new Timer();
    private double WantedRPM = 0;
    public SpoolPizzaDeliveryToRPM(DeliveryShooter pickupSpinner,double Rpmwanted){
        m_DeliveryShooter = pickupSpinner;
        WantedRPM = Rpmwanted;
        addRequirements(m_DeliveryShooter);
    }

    @Override
  public void initialize() {
    m_Timer.reset();
    m_Timer.start();
    m_DeliveryShooter.SetShootSpeed(WantedRPM);
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
    //THESE CHECK AGAINST THE ACTUAL SET RPM NOT THE WANTED RPM IN THIS COMMAND!
    boolean isUpperWithinRange = Constants.isWithinPercentage(m_DeliveryShooter.CurrentEncoderVelocity, m_DeliveryShooter.LastSetRPM, percentageTolerance);
    boolean islowerWithinRange = Constants.isWithinPercentage(m_DeliveryShooter.CurrentEncoderVelocity_LowS, m_DeliveryShooter.LastSetRPM, percentageTolerance);
    SmartDashboard.putBoolean("isUpperWithinRange", isUpperWithinRange);
    SmartDashboard.putBoolean("islowerWithinRange", islowerWithinRange);
    if (islowerWithinRange & isUpperWithinRange) {
         return true;
    }  
    return false;
  }
}
