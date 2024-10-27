package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Helper.Arm;
import org.firstinspires.ftc.teamcode.Helper.SMM;
import org.firstinspires.ftc.teamcode.Helper.Viper;

//make the methods into objects so you can call them
@Disabled
@Autonomous(name="Holding Straight", group="Autonomous")
public class Holding_Straight_Test extends LinearOpMode{

        /* Declare OpMode members. */
        DcMotor arm;
        private DcMotor         FrontLeft;
        private DcMotor         FrontRight;
        private DcMotor         RearLeft;
        private DcMotor         RearRight;
        private IMU imu = null;       // Control/Expansion Hub IMU

        private double          headingError  = 0;

        // These variable are declared here (as class members) so they can be updated in various methods,
        // but still be displayed by sendTelemetry()
        private double  targetHeading = 0;
        private double  driveSpeed    = 0;
        private double  turnSpeed     = 0;
        private double  FrontLeftSpeed     = 0;
        private double  FrontRightSpeed    = 0;
        private double  RearLeftSpeed     = 0;
        private double  RearRightSpeed    = 0;
        private int     FrontLeftTarget    = 0;
        private int     FrontRightTarget   = 0;
        private int     RearLeftTarget    = 0;
        private int     RearRightTarget   = 0;
        // Calculate the COUNTS_PER_INCH for your specific drive train.
        // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
        // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
        // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
        // This is gearing DOWN for less speed and more torque.
        // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
        static final double     COUNTS_PER_MOTOR_REV    = 537.7 ;   // eg: GoBILDA 312 RPM Yellow Jacket
        static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
        static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
        static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                (WHEEL_DIAMETER_INCHES * 3.1415);

        // These constants define the desired driving/control characteristics
        // They can/should be tweaked to suit the specific robot drive train.
        static final double     DRIVE_SPEED             = 0.4;     // Max driving speed for better distance accuracy.
        static final double     TURN_SPEED              = 0.2;     // Max Turn speed to limit turn rate
        static final double     HEADING_THRESHOLD       = 1.0 ;    // How close must the heading get to the target before moving to next step.
        // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
        // Define the Proportional control coefficient (or GAIN) for "heading control".
        // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
        // Increase these numbers if the heading does not corrects strongly enough (eg: a heavy robot or using tracks)
        // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
        static final double     P_TURN_GAIN            = 0.02;     // Larger is more responsive, but also less stable
        static final double     P_DRIVE_GAIN           = 0.03;     // Larger is more responsive, but also less stable
    RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;
    RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;
    RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
    static final int TICKS_PER_MOTOR_REV = 1425;
    static final int TICKS_PER_DEGREE = TICKS_PER_MOTOR_REV / 120;
    final double TICKS_PER_DEGREES = 12;  // Adjust this value as necessary
    private Viper Viper;
    private SMM SMM;
    private Arm Arm;
    @Override
    public void runOpMode() {
        initHardware();
        Viper = new Viper(hardwareMap);
        SMM = new SMM(hardwareMap);
        Arm = new Arm(hardwareMap);
        //distance to strafe a park is how far to move during autonomous to park the robot
        //to make it easier, the first number is just a placeholder

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();
        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();
        while (opModeIsActive()) {
            turnToHeading(1, 0);
            driveStraight(1, 10, 0);
            driveStraight(1, -10, 0);
            turnToHeading(1, 270);
            turnToHeading(1, 90);
            turnToHeading(1, 330);
            driveStraight(1, 10, 330);
            /*
            turnToHeading(0.5, 90);
            driveStraight(0.5, 5, 90);
            driveStraight(0.5, -10, 0);
            turnToHeading(0.5, 90);
            driveStraight(0.5,10,0);

             */
        }
    }


    public void driveStraight(double maxDriveSpeed,
                              double distance,
                              double heading) {

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            int moveCounts = (int)(distance * COUNTS_PER_INCH);
            FrontLeftTarget = FrontLeft.getCurrentPosition() + moveCounts;
            FrontRightTarget = FrontRight.getCurrentPosition() + moveCounts;
            RearLeftTarget = FrontLeft.getCurrentPosition() + moveCounts;
            RearRightTarget = FrontRight.getCurrentPosition() + moveCounts;

            // Set Target FIRST, then turn on RUN_TO_POSITION
            FrontLeft.setTargetPosition(FrontLeftTarget);
            FrontRight.setTargetPosition(FrontRightTarget);
            RearLeft.setTargetPosition(RearLeftTarget);
            RearRight.setTargetPosition(RearRightTarget);

            FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            moveRobot(maxDriveSpeed, 0);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (FrontLeft.isBusy() && FrontRight.isBusy() && RearLeft.isBusy() && RearRight.isBusy())) {

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    turnSpeed *= -1.0;

                // Apply the turning correction to the current driving speed.
                moveRobot(driveSpeed, turnSpeed);

                // Display drive status for the driver.
                sendTelemetry(true);
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            moveRobot(0, 0);
            FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     *  Spin on the central axis to point in a new direction.
     *  <p>
     *  Move will stop if either of these conditions occur:
     *  <p>
     *  1) Move gets to the heading (angle)
     *  <p>
     *  2) Driver stops the OpMode running.
     *
     * @param maxTurnSpeed Desired MAX speed of turn. (range 0 to +1.0)
     * @param heading Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *              If a relative angle is required, add/subtract from current heading.
     */
    public void turnToHeading(double maxTurnSpeed, double heading) {

        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, P_DRIVE_GAIN);

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(false);
        }

        // Stop all motion;
        moveRobot(0, 0);
    }

    /**
     *  Obtain & hold a heading for a finite amount of time
     *  <p>
     *  Move will stop once the requested time has elapsed
     *  <p>
     *  This function is useful for giving the robot a moment to stabilize it's heading between movements.
     *
     * @param maxTurnSpeed      Maximum differential turn speed (range 0 to +1.0)
     * @param heading    Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void holdHeading(double maxTurnSpeed, double heading, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        // keep looping while we have time remaining.
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(false);
        }

        // Stop all motion;
        moveRobot(0, 0);
    }

    // **********  LOW Level driving functions.  ********************

    /**
     * Use a Proportional Controller to determine how much steering correction is required.
     *
     * @param desiredHeading        The desired absolute heading (relative to last heading reset)
     * @param proportionalGain      Gain factor applied to heading error to obtain turning power.
     * @return                      Turning power needed to get to required heading.
     */
    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry

        // Determine the heading current error
        headingError = targetHeading - getHeading();

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    /**
     * Take separate drive (fwd/rev) and turn (right/left) requests,
     * combines them, and applies the appropriate speed commands to the left and right wheel motors.
     * @param drive forward motor speed
     * @param turn  clockwise turning motor speed.
     */
    public void moveRobot(double drive, double turn) {
        driveSpeed = drive;     // save this value as a class member so it can be used by telemetry.
        turnSpeed  = turn;      // save this value as a class member so it can be used by telemetry.

        FrontLeftSpeed  = drive - turn;
        FrontRightSpeed = drive + turn;
        RearLeftSpeed  = drive - turn;
        RearRightSpeed = drive + turn;

        // Scale speeds down if either one exceeds +/- 1.0;
        double max1 = Math.max(Math.abs(FrontLeftSpeed), Math.abs(FrontRightSpeed));
        double max2 = Math.max(Math.abs(RearRightSpeed), Math.abs(RearLeftSpeed));

        if (max1 > 1.0 || max2 > 1.0)
        {
            FrontLeftSpeed /= max1;
            FrontRightSpeed /= max1;
            RearLeftSpeed /= max2;
            RearRightSpeed /= max2;
        }



        FrontLeft.setPower(FrontLeftSpeed);
        FrontRight.setPower(FrontRightSpeed);
        RearLeft.setPower(RearLeftSpeed);
        RearRight.setPower(RearRightSpeed);
    }

    /**
     *  Display the various control parameters while driving
     *
     * @param straight  Set to true if we are driving straight, and the encoder positions should be included in the telemetry.
     */
    private void sendTelemetry(boolean straight) {

        if (straight) {
            telemetry.addData("Motion", "Drive Straight");
            telemetry.addData("Target Pos L:R",  "%7d:%7d",      FrontLeftTarget,  FrontRightTarget, RearLeftTarget,  RearRightTarget);
            telemetry.addData("Actual Pos L:R",  "%7d:%7d",      FrontLeft.getCurrentPosition(),
                    FrontRight.getCurrentPosition(), RearLeft.getCurrentPosition(), RearRight.getCurrentPosition());
        } else {
            telemetry.addData("Motion", "Turning");
        }

        telemetry.addData("Heading- Target : Current", "%5.2f : %5.0f", targetHeading, getHeading());
        telemetry.addData("Error  : Steer Pwr",  "%5.1f : %5.1f", headingError, turnSpeed);
        telemetry.addData("Wheel Speeds L : R", "%5.2f : %5.2f", FrontLeftSpeed, FrontRightSpeed, RearRightSpeed, RearLeftSpeed);
        telemetry.update();
    }

    /**
     * read the Robot heading directly from the IMU (in degrees)
     */
    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }
    private void initHardware() {
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must match the names assigned during the robot configuration.
        // step (using the FTC Robot Controller app on the phone).
        FrontLeft = hardwareMap.get(DcMotor.class, "Left_front");
        FrontRight = hardwareMap.get(DcMotor.class, "Right_front");
        RearLeft = hardwareMap.get(DcMotor.class, "Left_rear");
        RearRight = hardwareMap.get(DcMotor.class, "Right_rear");



        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        FrontLeft.setDirection(DcMotor.Direction.REVERSE);
        RearLeft.setDirection(DcMotor.Direction.REVERSE);
        FrontRight.setDirection(DcMotor.Direction.FORWARD);
        RearRight.setDirection(DcMotor.Direction.FORWARD);

        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

}





