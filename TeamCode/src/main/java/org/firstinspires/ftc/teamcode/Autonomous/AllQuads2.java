package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Components.Drivetrain;
import org.firstinspires.ftc.teamcode.Components.QuadrantPot;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.teamcode.Helper.SMM;
import org.firstinspires.ftc.teamcode.Helper.Viper;

import org.firstinspires.ftc.teamcode.Helper.Arm;
@Autonomous(name="AllQuads2", group="Autonomous")
@Disabled
public class AllQuads2 extends LinearOpMode {
    //components
    Viper Viper = new Viper(hardwareMap);
    SMM SMM = new SMM(hardwareMap);
    Arm Arm = new Arm(hardwareMap);
    final Drivetrain m_Drivetrain = new Drivetrain();
    public com.qualcomm.robotcore.hardware.DistanceSensor sensorLeft;
    public com.qualcomm.robotcore.hardware.DistanceSensor sensorRight;
    final QuadrantPot m_QuadrantPot = new QuadrantPot();
    final double DESIRED_DISTANCE = 13.5; //  this is how close the camera should get to the target (inches)
    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    public final double SPEED_GAIN = 0.02;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    public final double STRAFE_GAIN = 0.015;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    public final double TURN_GAIN = 0.01;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
    public final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)
    public final double DETECTION_THRESHOLD = 20.0;  // e.g., 20 cm this is used for detecting the prop
    public static final double DETECTION_THRESHOLD2 = 12.7;  // e.g., 12.7 cm thi is used to see if there is another robot in the path while strafing
    public boolean objectDetectedLeft = false; // this is for looking for the prop on spike mark
    boolean detectProp = true; // this says we have detected the prop and are performing actions related to having found the prop
    public boolean objectDetectedRight = false;// this is for looking for the prop on spike mark
    protected boolean lookForProp = false; // this tells the robot to look for the prop when true, while driving forward
    public boolean lookForRobot =false;
    boolean atBackdrop = false;
    boolean ENDGAME = false;
    TouchSensor touchSensor;  // Touch sensor Object
    boolean startOfAutonomous = true;
    //These are for setting the speed of the robot
    double fwdPwr = 0.6; //how fast to mover forward
    double bwdPwr = 0.6; //how fast to move backward
    double strafePwr= 0.6; //how fast to strafe
    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private int DESIRED_TAG_ID;     // Choose the tag you want to approach or set to -1 for ANY tag.
    public VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection detectedTag = null;     // Used to hold the data for a detected AprilTag
    boolean findAprilTag = false;
    int Position;
    private Servo pmmA;
    private Servo pmmF;
    static final int TICKS_PER_MOTOR_REV = 1425; //both servos pmmA and pmmF have the same resolution
    public static final int TICKS_PER_DEGREE = TICKS_PER_MOTOR_REV / 120; // there is a gear that have a 3 to 1 ratio
    RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;
    RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;
    RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
    private IMU imu = null;      // Control/Expansion Hub IMU

    @Override
    public void runOpMode() {
        // Step 1: Initialization
        initHardware();
        int quadrant = m_QuadrantPot.getPotentiometerQuadrant();
        boolean targetFound = false;    // APRIL TAG: Set to true when an AprilTag target is detected
        double drive = 0;        // Desired forward power/speed (-1 to +1)
        double strafe = 0;        // Desired strafe power/speed (-1 to +1)
        double turn = 0;        // Desired turning power/speed (-1 to +1)

       while (opModeInInit()) {
            telemetry.addData(">", "Robot Heading = %4.0f", getHeading());
            telemetry.update();
        }

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        while (opModeIsActive()) {
            // when the Robot is at the backdrop and aligned to the correct April Tag

            //Proceed to Parking the robot
            //THIS IS THE START OF AUTONOMOUS
            if (startOfAutonomous) {
                //The robot is at the wall at the start of Autonomous. Remember that the direction of the IMU sets the 0 and the start of each run.
                m_Drivetrain.turnToHeading(0);
                //initially the robot has the robot facing towards the wall so we have to move the robot backwards
                //we want to get past the truss before we start looking for the prop
                m_Drivetrain.driveBackward(21);
                //drive backwards until you find where the prop is located, the distance sensors detect left and right and if neither then it is front
                m_Drivetrain.driveBackward(11);
                //stop looking for the prop
                m_Drivetrain.driveForward(6);
                startOfAutonomous = false;
            }
        }
    }

     private void initHardware() {
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must match the names assigned during the robot configuration.
        // step (using the FTC Robot Controller app on the phone).
     //   m_Drivetrain.InitDrivetrain();
      //  m_QuadrantPot.InitQuadrantPot();

        // only implement this code if you have the touch sensor driving where to park
        touchSensor = hardwareMap.get(TouchSensor.class, "touch");
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();
        sensorLeft = hardwareMap.get(com.qualcomm.robotcore.hardware.DistanceSensor.class, "LeftD"); // Replace 'sensorLeftName' with the name you've given the sensor in the configuration.
        sensorRight = hardwareMap.get(com.qualcomm.robotcore.hardware.DistanceSensor.class, "RightD"); // Replace 'sensorLeftName' with the name you've given the sensor in the configuration.

    }

    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }
}
