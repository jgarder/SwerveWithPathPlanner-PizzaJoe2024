package frc.robot.commands;
import com.revrobotics.CANSparkBase;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer.PizzaManager;
import frc.robot.subsystems.DeliveryHolder;
import frc.robot.subsystems.DrivetrainManager;
public class ForwardBump extends Command{
    private final DrivetrainManager drivetrainManager;
    private final Timer m_Timer = new Timer();
    double Timeout = .5;
    public ForwardBump(DrivetrainManager dTM,double timeout){
        drivetrainManager = dTM;
        Timeout = timeout;
        addRequirements(drivetrainManager.drivetrain);
    }

    @Override
  public void initialize() {
    //PizzaManager.NoteInDeliveryHolder = false;
    m_Timer.reset();
    m_Timer.start();
  }
  
  @Override
  public void execute() {
    drivetrainManager.drivetrain.setControl(drivetrainManager.RobotCentricdrive.withVelocityX(.1 * drivetrainManager.MaxSpeed) // Drive forward with // negative Y (forward)
    .withVelocityY(0 * drivetrainManager.MaxSpeed) // Drive left with negative X (left)
    .withRotationalRate(0 * drivetrainManager.MaxAngularRate) // Drive counterclockwise with negative X (left)
    );
    //System.out.println("executing Undo");
    //m_DeliveryHolder.SetToWantedDutyCycle(1);
   //m_DeliveryHolder.MovePosition(reduction);
  }

  
  @Override
  public boolean isFinished() {
    if(m_Timer.get() > Timeout){
      drivetrainManager.drivetrain.setControl(drivetrainManager.RobotCentricdrive.withVelocityX(0 * drivetrainManager.MaxSpeed) // Drive forward with // negative Y (forward)
            .withVelocityY(0 * drivetrainManager.MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(0 * drivetrainManager.MaxAngularRate) // Drive counterclockwise with negative X (left)
           );
        return true;
    } 
    return false;
  }
}
