package frc.robot.commands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DeliveryTilt;

public class MoveDTiltToPosition extends Command{
    //private final PickupSpinner m_PickupSpinner;
    private final DeliveryTilt m_DeliveryTilter;
    private final Timer m_Timer = new Timer();
    double desiredSetpoint = 0;
    public MoveDTiltToPosition(double DesiredSetpoint, DeliveryTilt DelivTilt)
    {
        //m_PickupSpinner = pickupSpinner;
        m_DeliveryTilter = DelivTilt;
        this.desiredSetpoint = DesiredSetpoint;
        //System.out.printf("this.desiredSetpoint : " + this.desiredSetpoint);
        //addRequirements(m_PickupSpinner);
        addRequirements(m_DeliveryTilter);
    }
    public MoveDTiltToPosition(double DesiredSetpoint, DeliveryTilt DelivTilt,double tiltTolerance, double tiltSettleTime)
    {
        SetpointToleranceChosen = tiltTolerance;
        SettleTimeChosen = tiltSettleTime;
        m_DeliveryTilter = DelivTilt;
        this.desiredSetpoint = DesiredSetpoint;
        addRequirements(m_DeliveryTilter);
    }
    public MoveDTiltToPosition(Boolean TrapFloorOnlyDoNotUse, DeliveryTilt DelivTilt)
    {
        m_DeliveryTilter = DelivTilt;
        this.desiredSetpoint = SmartDashboard.getNumber("TrapFloorTilt",0);
        addRequirements(m_DeliveryTilter);
    }
    @Override
  public void initialize() {
    m_Timer.reset();
    m_Timer.start();
    m_DeliveryTilter.resetSettleTimer();
    m_DeliveryTilter.setSetpointToPosition(desiredSetpoint);
  }

  @Override
  public void execute() {
   
  }
  //.15;
  double TimeoutSeconds = 10.0;

  double AutonSettleTimeAtCorrectTilt = 0.0;
  double autonTolerance = Constants.DeliveryHead.TiltsetpointTolerance*2.5;

  boolean ReadyTofire = false;

  double SetpointToleranceChosen = Constants.DeliveryHead.TiltsetpointTolerance;
  double SettleTimeChosen = Constants.DeliveryHead.TiltSettleTimeAtPosition;
  @Override
  public boolean isFinished() {
    if(m_Timer.get() > TimeoutSeconds){
        return true;
    } 
    if(DriverStation.isAutonomousEnabled())
    {
      if (m_DeliveryTilter.atSetpoint(autonTolerance,AutonSettleTimeAtCorrectTilt)) 
      {      
        ReadyTofire = true;
        return true;
      }
    }
    else if (m_DeliveryTilter.atSetpoint(SetpointToleranceChosen,SettleTimeChosen)) 
    {
        ReadyTofire = true;
        return true;
    } 
    return false;
  }
}
