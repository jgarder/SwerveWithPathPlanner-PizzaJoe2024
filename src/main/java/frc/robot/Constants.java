package frc.robot;

public class Constants {

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
        public static final double Lift_maxValue = 430;
        

        public static final double Lift_Position_Zero = 0;
        public static final double Lift_Position_PullDown =5;
        public static final double Lift_Position_unfolding = 50;
        public static final double Lift_Position_CenterAndTrap = 429;
        public static final double Lift_Position_Edge = 429;


    }

    public static class PickupHead
    {
        //PID Constants for PICKUP Lifter
        public static final double kP_lifter = 0.05; 
        public static final double kI_lifter = 0.004;
        public static final double kD_lifter = 0.0033;

        //PID Constants for PICKUP Spinner
        public static final double kP_Spinner = 0.0250; 
        public static final double kI_Spinner = 0.0001;
        public static final double kD_Spinner = 0.005;

       

        public static final double spinnerspeed = .6;
        public static final double liftspeed = .5;

        public static final double spinnerSlewRate = 0.1;
        public static final double lifterSlewRate = 0.1;

        public static final double PickupZero = 0;
        public static final double PickupPassing = -1;
        public static final double PickupFloorPickup = 46;
        public static final double PickupSourcePickup = 13;
        public static final double PickupVertical = 16;
        
        //soft limits min/max values
        public static final double maxValue_Lifter = 44;
        public static final double minValue_Lifter = 0;
        public static final double maxValue_Spinner = 0;
        public static final double minValue_Spinner = 0;

    }
    public static class DeliveryHead
    {

      public static final double Lift_minValue = 0;
      public static final double Lift_maxValue = 133;
      

      public static final double Lift_Position_Zero = 0;
      public static final double Lift_Position_Passing = 15;
      public static final double Lift_Position_Speaker = 0;
      public static final double Lift_Position_Amp = 50;
      public static final double Lift_Position_HumanSource = 65;//50;
      public static final double Lift_Position_TrapStart = 120;
      public static final double Lift_Position_Trap = 133;
      public static final double Lift_Position_TrapShoot = 133;

      //TILT
      public static final double Tilt_minValue = 0;
      public static final double Tilt_maxValue = 28;
      
      public static final double Tilt_Position_Zero = 0;
      public static final double Tilt_Position_Park = 0;
      public static final double Tilt_Position_Passing = 5;
      public static final double Tilt_Position_TrapLift = 7.75;
      public static final double Tilt_Position_TrapLiftUp = 10.75;
      public static final double Tilt_Position_TrapLiftUpSHOOT = 28;
      public static final double Tilt_Position_Speaker_Closest = 8;
      public static final double Tilt_Position_Speaker_SafePost = 11.75;//10.85;
      public static final double Tilt_Position_Speaker_Furthest = 10;
      public static final double Tilt_Position_Amp = 22;

      public static final double Tilt_Position_TrapStart = 11;
      public static final double Tilt_Position_Trap = 12;
      public static final double Tilt_Position_HumanSource = 12;
      

      public static final double ShooterRpmOff = 0;
      public static final double ShooterRpmSpeakerClose = 4500;
      public static final double ShooterRpmSpeakerPodium = 5500;
      public static final double ShooterRpmAmp = 1500;
      public static final double ShooterRpmHumanSource = -500;
    }
    
    public static class AllianceAprilTags{
      public static class Red {
        public static final int SourceLeft = 10;//from robots pov
        public static final int SourceRight = 9;//from robots pov
        public static final int SpeakerCenter = 4;
        public static final int SpeakerOffset = 3;
        public static final int Amp = 5;
        public static final int ChainSourceSide = 11;
        public static final int ChainAmpSide = 12;
        public static final int ChainBackSide = 13;
      }
      public static class Blue {
        public static final int SourceLeft = 2;//from robots pov
        public static final int SourceRight = 1;//from robots pov
        public static final int SpeakerCenter = 7;
        public static final int SpeakerOffset = 8;
        public static final int Amp = 6;
        public static final int ChainSourceSide = 16;
        public static final int ChainAmpSide = 15;
        public static final int ChainBackSide = 14;
      }

    }
}
