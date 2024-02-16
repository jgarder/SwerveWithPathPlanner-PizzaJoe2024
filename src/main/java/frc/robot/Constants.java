package frc.robot;

public class Constants {

  public static class CANBus
  {
    //NEO CANBUS
    public static final String kRIOCANbusName = "rio";
    public static final int PickUpLifterCanBusID = 8;//7;
    public static final int PickUpSpinnerCanBusID = 15;//16;
    public static final int CANdleCanBusID = 11;
    public static final int UppershooterCanID = 14;
    public static final int LowerShooterCanID = 7;
    public static final int DeliveryIntakeCanBusID = 5;

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


    public static boolean isWithinPercentage(double numberA, double numberB, double range)
    {
        double percentage = 0;
        percentage = (numberB - numberA) * 100 / numberA;

        return Math.abs(percentage) <= range;
    }

    public static class NeoBrushless
    {
      public static int neo550safelimitAmps = 35;//Pickup rollers and delivery index motor
      public static int neo1650safelimitAmps = 30; //pickup head lifter, delivery Head lifter, delivery tilt motor, 2x shooting motor, Chain Lift motor
    }

    public static class PickupHead
    {
        //PID Constants for PICKUP Lifter
        public static final double kP_lifter = 0.0250; 
        public static final double kI_lifter = 0.00001;
        public static final double kD_lifter = 0.003;

        //PID Constants for PICKUP Spinner
        public static final double kP_Spinner = 0.040; 
        public static final double kI_Spinner = 0.0001;
        public static final double kD_Spinner = 0.005;

       

        public static final double spinnerspeed = .6;
        public static final double liftspeed = .5;

        public static final double spinnerSlewRate = 0.1;
        public static final double lifterSlewRate = 0.1;

        public static final double PickupZero = 0;
        public static final double PickupPassing = 0;
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
      public static final double Lift_Position_Passing = 0;
      public static final double Lift_Position_Speaker = 25;
      public static final double Lift_Position_Amp = 50;
      public static final double Lift_Position_TrapStart = 120;
      public static final double Lift_Position_Trap = 125;

      //TILT
      public static final double Tilt_minValue = 0;
      public static final double Tilt_maxValue = 19;
      
      public static final double Tilt_Position_Zero = 0;
      public static final double Tilt_Position_Passing = 6;
      public static final double Tilt_Position_Speaker = 5;
      public static final double Tilt_Position_Amp = 10;
      public static final double Tilt_Position_TrapStart = 11;
      public static final double Tilt_Position_Trap = 12;
      
    }
    
}
