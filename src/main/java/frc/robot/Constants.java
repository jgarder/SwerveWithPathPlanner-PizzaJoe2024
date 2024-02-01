package frc.robot;

public class Constants {

    public static final int CANdleID = 11;

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
        public static final double kP_Spinner = 0.0250; 
        public static final double kI_Spinner = 0.0001;
        public static final double kD_Spinner = 0.005;

        public static final int PickUpLifterCanBusID = 8;//7;
        public static final int PickUpSpinnerCanBusID = 15;//16;

        public static final double spinnerspeed = .5;
        public static final double liftspeed = .5;

        public static final double spinnerSlewRate = 0.1;
        public static final double lifterSlewRate = 0.1;

        public static final double PickupZero = 0;
        public static final double PickupFloorPickup = 44;
        public static final double PickupSourcePickup = 13;
        public static final double PickupVertical = 16;
        public static final double PickupPassing = 7;
        //soft limits min/max values
        public static final double maxValue_Lifter = 44;
        public static final double minValue_Lifter = 0;
        public static final float maxValue_Spinner = 0;
        public static final float minValue_Spinner = 0;

    }
    public static class DeliveryHead
    {


      
      public static final int LifterCanBusID = 10;

      public static final double minValue_Lifter = 0;
      public static final float maxValue_Lifter = 133;

      public static final double Position_Zero = 0;
      public static final double Position_Speaker = 25;
      public static final double Position_Amp = 50;
      public static final double Position_TrapStart = 120;
      public static final double Position_Trap = 125;
      
    }
    
}
