package frc.robot;

public class Constants {
    public static class NeoBrushless
    {
      public static int neo550safelimitAmps = 35;//Pickup rollers and delivery index motor
      public static int neo1650safelimitAmps = 45; //pickup head lifter, delivery Head lifter, delivery tilt motor, 2x shooting motor, Chain Lift motor
    }

    public static class PickupHead
    {
        //PID Constants for PICKUP Lifter
        public static final double kP_lifter = 0.0250; 
        public static final double kI_lifter = 0.0001;
        public static final double kD_lifter = 0.005;

        //PID Constants for PICKUP Spinner
        public static final double kP_Spinner = 0.0250; 
        public static final double kI_Spinner = 0.0001;
        public static final double kD_Spinner = 0.005;

        public static final int PickUpLifterCanBusID = 7;
        public static final int PickUpSpinnerCanBusID = 16;

        public static final double spinnerspeed = .5;
        public static final double liftspeed = .5;

        public static final double spinnerSlewRate = 0.1;
        public static final double lifterSlewRate = 0.1;

        public static final double lifterPickupPosition = 10;
        public static final double lifterSourcePickupPosition = 2;
        public static final double lifterverticalPosition = 5;
        public static final double lifterPassingPosition = 0;

        //soft limits min/max values
        public static final double maxValue_Lifter = 15;
        public static final double minValue_Lifter = 0;
        public static final float maxValue_Spinner = 0;
        public static final float minValue_Spinner = 0;

    }
}
