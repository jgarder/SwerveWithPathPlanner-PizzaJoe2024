package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.PizzaManager;

public class SmartDashboardHandler extends SubsystemBase{

private final SendableChooser<String> m_chooser = new SendableChooser<>();
RobotContainer thisrobot;


  public SmartDashboardHandler(RobotContainer mainbrain) {
    thisrobot = mainbrain;

    // Build an auto chooser. This will use Commands.none() as the default option.
    
    SmartDashboard.putNumber("TrapFloorTilt", Constants.DeliveryHead.Tilt_Position_TrapFloorShoot);
    SmartDashboard.putNumber("TrapFloorRPM", Constants.DeliveryHead.ShooterRpmTrapFloor);

    bootupPersistents();
    //BuildAutonomousChooser();
    //thisrobot.m_pdh.clearStickyFaults();
    //BuildConeCubeChooser();
  }
  public String XYSpeedName = "Jow Speed Multiplier";
  public String ZrotMultiplierName = "Jow Rotation Multiplier";
  public String LimelightbypassName = "Bypass Limelight";
  private void bootupPersistents() {

    if(!SmartDashboard.containsKey(XYSpeedName))
        {
            SmartDashboard.putNumber(XYSpeedName, PizzaManager.speedMulti);
            SmartDashboard.setPersistent(XYSpeedName);
        }
        if(!SmartDashboard.containsKey(ZrotMultiplierName))
        {
            SmartDashboard.putNumber(ZrotMultiplierName, PizzaManager.RotationMulti);
            SmartDashboard.setPersistent(ZrotMultiplierName);
        }
      if(!SmartDashboard.containsKey(LimelightbypassName))
      {
          SmartDashboard.putBoolean(LimelightbypassName, false);
          SmartDashboard.setPersistent(LimelightbypassName);
      }
  }

  // public String getChosenAutoString()
  // {
  //   return m_chooser.getSelected();
  // }
//   public static final String kDefaultAuto = "Default";
//   public static final String kCalibrateYesAuto = "Shoot Note Dont Move";
//   private void BuildAutonomousChooser() {
//     // build Autonomous selector. 
//     m_chooser.setDefaultOption("No Auto", kDefaultAuto);
//     m_chooser.addOption("Shoot Note Dont Move", kCalibrateYesAuto);
//     //m_chooser.addOption("Charge Pad Spin Move", kChargePadSpin);
//     SmartDashboard.putData("Auto choices", m_chooser);
//     //SmartDashboard.setPersistent("Auto choices");
    
// }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        getdataToDashboard();
        UpdatePDHToSmartDashboard();
    }

    public void getdataToDashboard()
    {
        if (DriverStation.getAlliance().isPresent()) {
          SmartDashboard.putString("Alliance", DriverStation.getAlliance().toString());
        }
        SmartDashboard.putBoolean("teleOpEnable", DriverStation.isTeleopEnabled());
        SmartDashboard.putBoolean("AutonEnable", DriverStation.isAutonomousEnabled());
        //SmartDashboard.putString("Alliance", DriverStation.getAlliance().toString());
        SmartDashboard.putBoolean("Pickup Spinner OverTemp", !thisrobot.pickupSpinner.isMotorOvertemp());
        
        PizzaManager.LimeLightBypassed =  SmartDashboard.getBoolean(LimelightbypassName, false);
        PizzaManager.RotationMulti = MathUtil.clamp(SmartDashboard.getNumber(ZrotMultiplierName, PizzaManager.RotationMulti),0,1);
        PizzaManager.speedMulti = MathUtil.clamp(SmartDashboard.getNumber(XYSpeedName, PizzaManager.speedMulti),0,1);

        Constants.DeliveryHead.Tilt_Position_TrapFloorShoot = SmartDashboard.getNumber("TrapFloorTilt",0);
        Constants.DeliveryHead.ShooterRpmTrapFloor = SmartDashboard.getNumber("TrapFloorRPM", 0);
        //System.out.printf("-tiltPos : " + Constants.DeliveryHead.Tilt_Position_TrapFloorShoot);
        //System.out.printf("-ShooterRpmTrapFloor : " + Constants.DeliveryHead.ShooterRpmTrapFloor);
    }
    
    private static final int NUM_PDH_CHANNELS =24;
    private double maxcurrent = 0.0;
  public double lowestVoltage = 48.00;
  public void UpdatePDHToSmartDashboard() {
     /**
     * Get the input voltage of the PDH and display it on Shuffleboard.
     */
    double Voltage = thisrobot.m_pdh.getVoltage();
    if (Voltage < lowestVoltage){
      lowestVoltage = Voltage;
    }
    SmartDashboard.putNumber("Voltage", Voltage);
    SmartDashboard.putNumber("Lowest Voltage", lowestVoltage);
    /**
     * Get the total current of the PDH and display it on Shuffleboard. This will
     * be to the nearest even number.
     *
     * To get a better total current reading, sum the currents of all channels.
     * See below for getting channel current.
     */
    double current = thisrobot.m_pdh.getTotalCurrent();
    if (current > maxcurrent){
      maxcurrent = current;
    }
    SmartDashboard.putNumber("Total Current",current);
    SmartDashboard.putNumber("MAX Total Current", maxcurrent);
    /**
     * Get the currents of each channel of the PDH and display them on
     * Shuffleboard.
     */
    // for (int channel = 0; channel < NUM_PDH_CHANNELS; channel++) {
    //   SmartDashboard.putNumber(
    //       ("Ch" + String.valueOf(channel) + " Current"),
    //       thisrobot.m_pdh.getCurrent(channel));
    // }
  }
}
