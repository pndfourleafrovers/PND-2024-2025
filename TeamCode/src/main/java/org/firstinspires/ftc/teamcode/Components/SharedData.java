package org.firstinspires.ftc.teamcode.Components;
public class SharedData {
    public boolean lookForProp = false;
    public boolean lookForRobot = false;
    public boolean objectDetectedLeft = false;
    public boolean objectDetectedRight = false;
    public boolean findAprilTag=false;
    public boolean atBackdrop = false;
    public static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    public final double MAX_AUTO_STRAFE = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    public final double MAX_AUTO_TURN = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)
    public final double Drive_Fwd_Minimum=0.2;
    public static double SPEED_GAIN = 0.02;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    public final double STRAFE_GAIN = 0.015;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    public final double TURN_GAIN = 0.01;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
    public static double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)


}
