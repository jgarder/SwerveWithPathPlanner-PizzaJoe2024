package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class Constants {
  public static final String LimelightName = "limelight";

    /*
	 * Talon FX has 2048 units per revolution
	 * 
	 * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#sensor-resolution
	 */
	public static final int kTalonFXUnitsPerRevolution = 2048; /* this is constant for Talon FX */

  public static final double nominalBatteryVoltage = 12.0;
  public static final double stickDeadband = 0.10;

  public static class XboxControllerMap {
    //THIS MAP EXCEPT POV exist as Xboxcontroller.button.KBack.value or .Axis.kRightY ect ect ect.
    //these are the pov directional buttons (this had 8 directions but we do not have them mapped.)
    public static final int kPOVDirectionUP = 0;//0 is up on xbox controller POV hat
    public static final int kPOVDirectionDOWN = 180;//180 is down on xbox controller POV hat
    public static final int kPOVDirectionRIGHT = 90;//90 is right on xbox controller POV hat
    public static final int kPOVDirectionLeft = 270;//270 is Left on xbox controller POV hat

  }

  public static class ChassisPid
  {
    public static final double k_PoseX_P = .4;//.5;//1.20;
    public static final double k_PoseX_I = .0000005;//0.000001;//0.02;
    public static final double k_PoseX_D = .06;//0.15;//0.0020;

    public static final double k_PoseY_P = .4;//.5;//1.20;
    public static final double k_PoseY_I = .0000005;//0.000001;//0.02;
    public static final double k_PoseY_D = .06;//0.15;//0.002; 

    public static final double k_RZ_P = 0.009000;//.05;
    public static final double k_RZ_I = 0.000000;//0.00;
    public static final double k_RZ_D = 0.000000;//0.00;

    public static final double minXposeErrorToCorrect = .06;
    public static final double minYposeErrorToCorrect = .06;
    public static final double minRZErrorToCorrect = 2;//1.25;

    public static final double min_xpose_command = 0.060;
    public static final double min_Ypose_command = 0.060;
    public static final double min_RZ_command = .055;

    //if we are really far away lets keep pid from going insane.
    public static final double maxYvelocity = .75;
    public static final double maxXvelocity = .75;
    public static final double maxRZvelocity = 3;

  }
  public static class TargetLocations
  {
    public static class Red
    {
       public static final double Xspeed = -1.0;//used to invert results with -1 or 1;
       public static final double Yspeed = -1.0;//used to invert results with -1 or 1;
       public static final double rotationspeed = 1.0;//used to invert results with -1 or 1;

      public static final Pose2d SpeakerCenter = new Pose2d(15.05, 5.5, Rotation2d.fromDegrees(0));

      public static final Pose2d SourceRight = new Pose2d(.60, 1.197, Rotation2d.fromDegrees(-120));//#9

      public static final Pose2d Amp = new Pose2d(14.74, 7.59, Rotation2d.fromDegrees(90));//#5

      public static final double StageAMPSIDE_XP_Setpoint = 12.1;//12.0;//4.52; //(Red stage Right)
      public static final double StageAMPSIDE_YP_Setpoint = 4.87;//4.83;//4.87; //(Red stage Right)
      public static final double StageAMPSIDE_RZ_Setpoint = -120;//-60; //(Red stage Right)
      public static final Pose2d ClimbChainStageAMPSIDE = new Pose2d(12.1, 4.87, Rotation2d.fromDegrees(-120));

      public static final Pose2d TrapFloorStageCenterSide = new Pose2d(10.4, 4.00, Rotation2d.fromDegrees(0));//13
      public static final Pose2d TrapFloorStageAmpSide = new Pose2d(12.27, 5.07, Rotation2d.fromDegrees(-120));//12
      public static final Pose2d TrapFloorStageSourceSide = new Pose2d(12.27, 2.92, Rotation2d.fromDegrees(120));//11
    
    }
    public static class Blue
    {
      public static final double Xspeed = 1.0;//used to invert results with -1 or 1;
       public static final double Yspeed = 1.0;//used to invert results with -1 or 1;
       public static final double rotationspeed = 1.0;//used to invert results with -1 or 1;

      public static final Pose2d SpeakerCenter = new Pose2d(1.45, 5.5, Rotation2d.fromDegrees(180));//BC-CALI:1.38,5.48,177

      public static final Pose2d SourceRight = new Pose2d(14.83, .577, Rotation2d.fromDegrees(-60));//#1

      public static final Pose2d Amp = new Pose2d(1.84, 7.59, Rotation2d.fromDegrees(90));//#6

      public static final double StageAMPSIDE_XP_Setpoint = 4.39;//4.52; //(blue stage left)
      public static final double StageAMPSIDE_YP_Setpoint = 4.80;//4.87; //(blue stage left)
      public static final double StageAMPSIDE_RZ_Setpoint = -62;//-60; //(blue stage left)

      public static final Pose2d TrapFloorStageCenterSide = new Pose2d(6.10, 4.00, Rotation2d.fromDegrees(-180));//14
      public static final Pose2d TrapFloorStageAmpSide = new Pose2d(4.25, 5.07, Rotation2d.fromDegrees(-60));//15
      public static final Pose2d TrapFloorSourceSide = new Pose2d(4.1910, 2.84, Rotation2d.fromDegrees(60));//16
    }
  }
  public static class CANBus
  {
    //NEO CANBUS
    public static final String kRIOCANbusName = "rio";
    public static final int PDH_CAN_ID = 1;
    public static final int PickUpLifterCanBusID = 8;//7;
    public static final int PickUpSpinnerCanBusID = 15;//16;
    public static final int CANdleCanBusID = 11;
    public static final int UppershooterCanID = 14;
    public static final int LowerShooterCanID = 7;
    public static final int DeliveryIntakeCanBusID = 5;
    public static final int ChainLifterCanBusID = 6;
    public static final int Lift_CanBusID = 10;
    public static final int Tilt_CanBusID = 9;

    //CANIVORE CANBUS
    public static final String kCANbusName = "8608ChassisCan";
    public static final int kPigeonCanId = 3;
    // Front Left
    public static final int kFrontLeftDriveMotorId = 19;
    public static final int kFrontLeftSteerMotorId = 21;
    public static final int kFrontLeftEncoderId = 29;
    // Front Right
    public static final int kFrontRightDriveMotorId = 23;
    public static final int kFrontRightSteerMotorId = 18;
    public static final int kFrontRightEncoderId = 28;
    // Back Left
    public static final int kBackLeftDriveMotorId = 25;
    public static final int kBackLeftSteerMotorId = 24;
    public static final int kBackLeftEncoderId = 26;
    // Back Right
    public static final int kBackRightDriveMotorId = 22;
    public static final int kBackRightSteerMotorId = 20;
    public static final int kBackRightEncoderId = 27;
  }


    public static boolean isWithinPercentage(double numberA, double numberB, double PercentRange)
    {
        double percentage = 0;
        percentage = (numberB - numberA) * 100 / numberA;

        return Math.abs(percentage) <= PercentRange;
    }
    public static boolean isWithinAmount(double numberA, double numberB, double NumericalTolerance)
    {
        double Difference = 0;
        Difference = numberB - numberA;

        return Math.abs(Difference) <= NumericalTolerance;
    }

    public static class NeoBrushless
    {
      public static int neo550safelimitAmps = 35;//Pickup rollers and delivery index motor
      public static int neo1650safelimitAmps = 30; //pickup head lifter, delivery Head lifter, delivery tilt motor, 2x shooting motor, Chain Lift motor
    }

    public static class ChainLifter
    {
        public static final double Lift_minValue = 0;
        public static final double Lift_maxValue = 340;//430;
        

        public static final double Lift_Position_Zero = 0;
        public static final double Lift_Position_PullDown =4;
        public static final double Lift_Position_unfolding = 30;
        public static final double Lift_Position_ForDeliveryKick = 270;
        public static final double Lift_Position_CenterAndTrap = 340;
        public static final double Lift_Position_Edge = 340;


    }

    public static class PizzaFloorPickupHead
    {
        //PID Constants for PICKUP Lifter
        public static final double kP_FloorPickupArm = 0.05; 
        public static final double kI_FloorPickupArm = 0.00001;
        public static final double kD_FloorPickupArm = 0.00;

        //PID Constants for PICKUP Spinner
        public static final double kP_Spinner = 0.0250; 
        public static final double kI_Spinner = 0.0001;
        public static final double kD_Spinner = 0.005;

        public static final double spinnerSlewRate = 0.1;
        public static final double lifterSlewRate = 0.1;

        //PickupArm 
        public static final double maxValue_PickupArm = 46;
        public static final double minValue_PickupArm = 0;
        public static final double PickupZero = 0;
        public static final double PickupPassing = 0;
        public static final double PickupFloorPickup = 46;
        public static final double PickupSourcePickup = 13;
        public static final double PickupVertical = 16;
        
        //soft limits min/max values
     
        public static final double maxValue_Spinner = 0;
        public static final double minValue_Spinner = 0;

    }
    public static class DeliveryHead
    {

      public static final double Lift_minValue = 0;
      public static final double Lift_maxValue = 140;
      

      public static final double Lift_Position_Zero = 0;
      public static final double Lift_Position_Passing = 15;
      public static final double Lift_Position_Speaker = 0;
      public static final double Lift_Position_Amp = 50;//50;
      public static final double Lift_Position_HumanSource = 100;//100;//85;//65;//50;
      public static final double Lift_Position_TrapStart = 120;
      public static final double Lift_Position_Trap = 140;
      public static final double Lift_Position_TrapShoot = 140;

      //TILT
      public static final double Tilt_minValue = 0;
      public static final double Tilt_maxValue = 28;
      
      public static final double Tilt_Position_Zero = 0;
      public static final double Tilt_Position_Park = 0;
      public static final double Tilt_Position_Passing = 5;
      public static final double Tilt_Position_TrapLift = 2;//7.75;
      public static final double Tilt_Position_TrapDodge = 6;//7.75;
      public static final double Tilt_Position_TrapLiftHitHook = 7.75;
      public static final double Tilt_Position_TrapLiftUp = 10.75;
      public static final double Tilt_Position_TrapLiftUpSHOOT = 28;
      public static final double Tilt_Position_Speaker_Closest = 7.4;
      public static final double Tilt_Position_Speaker_Podium = 11.6;//10.85;
      public static final double Tilt_Position_Speaker_Furthest = 10;
      public static final double Tilt_Position_Amp = 24;

      //public static final double Tilt_Position_TrapStart = 11;
      //public static final double Tilt_Position_Trap = 12;
      public static final double Tilt_Position_HumanSource = 16;//12;
      
      public static final double maxRPM = 6250;
      public static final double ShooterRpmOff = 0;
      public static final double ShooterRpmSpeakerClose = 4750;//4500;
      public static final double ShooterRpmSpeakerPodium = 6000;//5650;
      public static final double ShooterRpmAmp = 1500;
      public static final double ShooterRpmHumanSource = -1550;//-2250;//-1550;//-500;
    }
    
    public static class AllianceAprilTags{
      public static class Red {
        public static final int SourceLeft = 10;//from robots pov
        public static final int SourceRight = 9;//from robots pov
        public static final int SpeakerCenter = 4;
        public static final int SpeakerOffset = 3;
        public static final int Amp = 5;
        public static final int StageSourceSide = 11;
        public static final int StageAmpSide = 12;
        public static final int StageCenterSide = 13;
      }
      public static class Blue {
        public static final int SourceLeft = 2;//from robots pov
        public static final int SourceRight = 1;//from robots pov
        public static final int SpeakerCenter = 7;
        public static final int SpeakerOffset = 8;
        public static final int Amp = 6;
        public static final int StageSourceSide = 16;
        public static final int StageAmpSide = 15;
        public static final int StageCenterSide = 14;
      }

    }
}
