package frc.robot.commands;
import com.ctre.phoenix.led.CANdle;
import com.revrobotics.CANSparkBase;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer.PizzaManager;
import frc.robot.subsystems.CANdleSystem;
import frc.robot.subsystems.DeliveryHolder;
import frc.robot.subsystems.DrivetrainManager;
public class FlashGreenLights extends Command{
    private final CANdleSystem m_candleSubsystem;
    private final Timer m_Timer = new Timer();
    double Timeout = 2;
    public FlashGreenLights(CANdleSystem Subsys,double timeout){
        m_candleSubsystem = Subsys;
        Timeout = timeout;
        addRequirements(m_candleSubsystem);
    }

    @Override
  public void initialize() {
    //PizzaManager.NoteInDeliveryHolder = false;
    m_Timer.reset();
    m_Timer.start();
    m_candleSubsystem.StrobeGreenLights();
  }
  
  @Override
  public void execute() {
    
  }

  
  @Override
  public boolean isFinished() {
    if(m_Timer.get() > Timeout){
      m_candleSubsystem.OFFLights();
        return true;
    } 
    return false;
  }
}
