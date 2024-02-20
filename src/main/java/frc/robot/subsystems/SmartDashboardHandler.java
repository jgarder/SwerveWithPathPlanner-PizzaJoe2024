package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class SmartDashboardHandler extends SubsystemBase{

private final SendableChooser<String> m_chooser = new SendableChooser<>();
RobotContainer thisrobot;
public static final double defaultspeedMulti = .25;
public static final double defaultRotationMulti = .25;
  public SmartDashboardHandler(RobotContainer mainbrain) {
    thisrobot = mainbrain;

    bootupPersistents();
    BuildAutonomousChooser();
    //BuildConeCubeChooser();
  }

  private void bootupPersistents() {

    if(!SmartDashboard.containsKey("Jow Speed Multiplier"))
        {
            SmartDashboard.putNumber("Jow Speed Multiplier", defaultspeedMulti);
            SmartDashboard.setPersistent("Jow Speed Multiplier");
        }
        if(!SmartDashboard.containsKey("Jow Rotation Multiplier"))
        {
            SmartDashboard.putNumber("Jow Rotation Multiplier", defaultRotationMulti);
            SmartDashboard.setPersistent("Jow Rotation Multiplier");
        }
  }

  public String getChosenAutoString()
  {
    return m_chooser.getSelected();
  }
  public static final String kDefaultAuto = "Default";
  public static final String kCalibrateYesAuto = "Shoot Note Dont Move";
  private void BuildAutonomousChooser() {
    // build Autonomous selector. 
    m_chooser.setDefaultOption("No Auto", kDefaultAuto);
    m_chooser.addOption("Shoot Note Dont Move", kCalibrateYesAuto);
    //m_chooser.addOption("Charge Pad Spin Move", kChargePadSpin);
    SmartDashboard.putData("Auto choices", m_chooser);
    //SmartDashboard.setPersistent("Auto choices");
    
}

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        getdataToDashboard();
    }

    public void getdataToDashboard()
    {
        SmartDashboard.putBoolean("teleOpEnable", DriverStation.isTeleopEnabled());
        SmartDashboard.putBoolean("AutonEnable", DriverStation.isAutonomousEnabled());
        SmartDashboard.putString("Alliance", DriverStation.getAlliance().toString());
        SmartDashboard.putBoolean("Pickup Spinner OverTemp", !thisrobot.pickupSpinner.isMotorOvertemp());
        
    }
}
