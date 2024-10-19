package org.firstinspires.ftc.teamcode.Helper;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
@Disabled

public class OdometryMovement {
    // Declare motors and odometry encoders
    private DcMotor FrontLeft, RearLeft, FrontRight, RearRight;
    private DcMotor leftOdometry, rightOdometry, horizontalOdometry;

    // Variables for tracking the robot's position
    private double robotX = 0, robotY = 0, robotHeading = 0;
    private double lastLeftEncoder = 0, lastRightEncoder = 0, lastHorizontalEncoder = 0;

    // Encoder counts per inch for the wheels and encoders
    private final double ticksPerWheelInch = (537.7 / ((96 / 25.4) * Math.PI)); // For driving wheels
    private final double ticksPerEncoderInch = (2000 / ((48 / 25.4) * Math.PI)); // For odometry wheels

    // Distances between the odometry wheels and the center of rotation
    private double lateralDistance; // Distance between left and right odometry wheels
    private double horizontalWheelOffset; // Offset from the center to the horizontal wheel

    // Constructor to initialize the motors, encoders, and default distances
    public OdometryMovement(HardwareMap hardwareMap) {
        // Initialize with default values (in inches)
        this(hardwareMap, 14.0, 7.0); // Default lateral distance: 14 inches, horizontal offset: 7 inches
    }
// Lateral Distance: Distance between left and right odometry wheels (typically parallel to the robot's forward direction).
//Horizontal Offset: Distance from the robot's center to the horizontal wheel (which tracks side-to-side motion).
    // Constructor to initialize the motors, encoders, and allow customized distances
    //                  Lateral Distance (left-right)
//         Left Wheel --------------- Right Wheel
//               |                             |
//               |                             |
//               |     (center of rotation)    | <-- Horizontal Offset
//               |                             |
//               |                             |
//         Horizontal Wheel (perpendicular to left and right)
    public OdometryMovement(HardwareMap hardwareMap, double lateralDistance, double horizontalWheelOffset) {
        // Initialize the drive motors
        FrontLeft = hardwareMap.get(DcMotor.class, "Left_front");
        RearLeft = hardwareMap.get(DcMotor.class, "Left_rear");
        FrontRight = hardwareMap.get(DcMotor.class, "Right_front");
        RearRight = hardwareMap.get(DcMotor.class, "Right_rear");

        // Initialize the odometry encoders
        leftOdometry = hardwareMap.get(DcMotor.class, "leftOd");
        rightOdometry = hardwareMap.get(DcMotor.class, "rightOd");
        horizontalOdometry = hardwareMap.get(DcMotor.class, "centerOd");

        // Set motor directions
        FrontLeft.setDirection(DcMotor.Direction.REVERSE);
        RearLeft.setDirection(DcMotor.Direction.REVERSE);
        FrontRight.setDirection(DcMotor.Direction.FORWARD);
        RearRight.setDirection(DcMotor.Direction.FORWARD);

        // Set the lateral distance and horizontal wheel offset
        this.lateralDistance = lateralDistance;
        this.horizontalWheelOffset = horizontalWheelOffset;

        // Reset the encoders
        resetOdometryEncoders();
    }

    // Reset odometry encoders
    public void resetOdometryEncoders() {
        leftOdometry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightOdometry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalOdometry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftOdometry.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightOdometry.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontalOdometry.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lastLeftEncoder = 0;
        lastRightEncoder = 0;
        lastHorizontalEncoder = 0;
    }

    // Method to stop all motors
    public void stopMotors() {
        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        RearLeft.setPower(0);
        RearRight.setPower(0);
    }

    // Update the robot's position using odometry encoders and distances
    public void updateOdometry() {
        double leftEncoder = leftOdometry.getCurrentPosition();
        double rightEncoder = rightOdometry.getCurrentPosition();
        double horizontalEncoder = horizontalOdometry.getCurrentPosition();

        double deltaLeft = leftEncoder - lastLeftEncoder;
        double deltaRight = rightEncoder - lastRightEncoder;
        double deltaHorizontal = horizontalEncoder - lastHorizontalEncoder;

        // Convert encoder ticks to inches
        double deltaLeftInches = deltaLeft / ticksPerEncoderInch;
        double deltaRightInches = deltaRight / ticksPerEncoderInch;
        double deltaHorizontalInches = deltaHorizontal / ticksPerEncoderInch;

        // Calculate the change in forward movement and rotation
        double deltaForward = (deltaLeftInches + deltaRightInches) / 2.0;
        double deltaRotation = (deltaRightInches - deltaLeftInches) / lateralDistance;

        // Update the robot's heading
        robotHeading += deltaRotation;

        // Update the robot's X and Y positions considering both forward and lateral movements
        robotX += deltaForward * Math.cos(Math.toRadians(robotHeading)) + (deltaHorizontalInches - deltaRotation * horizontalWheelOffset);
        robotY += deltaForward * Math.sin(Math.toRadians(robotHeading));

        // Store the current encoder values for the next update
        lastLeftEncoder = leftEncoder;
        lastRightEncoder = rightEncoder;
        lastHorizontalEncoder = horizontalEncoder;
    }

    // Method to move the robot to a specified (x, y) position
    public void moveToPosition(double targetX, double targetY, double power) {
        while (Math.abs(targetX - robotX) > 1 || Math.abs(targetY - robotY) > 1) {
            updateOdometry();

            double deltaX = targetX - robotX;
            double deltaY = targetY - robotY;

            double distanceToTarget = Math.hypot(deltaX, deltaY);
            double angleToTarget = Math.toDegrees(Math.atan2(deltaY, deltaX));

            // Calculate movement direction relative to the robot's heading
            double headingError = angleToTarget - robotHeading;

            // Normalize the heading error between -180 and 180 degrees
            headingError = (headingError + 360) % 360;
            if (headingError > 180) headingError -= 360;



       //     FrontLeft.setTargetPosition(FrontLeft.getCurrentPosition() + ticksToMove);
        //    FrontRight.setTargetPosition(FrontRight.getCurrentPosition() + ticksToMove);
         //   RearLeft.setTargetPosition(RearLeft.getCurrentPosition() + ticksToMove);
        //    RearRight.setTargetPosition(RearRight.getCurrentPosition() + ticksToMove);

            // Set the mode to RUN_TO_POSITION
            FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);








            // Calculate motor powers
            double forwardPower = power * Math.cos(Math.toRadians(headingError));
            double strafePower = power * Math.sin(Math.toRadians(headingError));

            // Set motor powers
            FrontLeft.setPower(forwardPower + strafePower);
            FrontRight.setPower(forwardPower - strafePower);
            RearLeft.setPower(forwardPower - strafePower);
            RearRight.setPower(forwardPower + strafePower);
        }

        // Stop the motors when the target is reached
        stopMotors();
    }

    // Method to rotate the robot to a specific heading (in degrees)
    public void rotateToHeading(double targetHeading, double power) {
        while (Math.abs(targetHeading - robotHeading) > 1) {
            updateOdometry();

            double headingError = targetHeading - robotHeading;

            // Normalize the heading error between -180 and 180 degrees
            headingError = (headingError + 360) % 360;
            if (headingError > 180) headingError -= 360;

            double turnPower = power * Math.signum(headingError);

            // Set motor powers for turning
            FrontLeft.setPower(-turnPower);
            FrontRight.setPower(turnPower);
            RearLeft.setPower(-turnPower);
            RearRight.setPower(turnPower);
        }

        // Stop the motors when the target heading is reached
        stopMotors();
    }

    // Optional getters for robot position and heading (for debugging)
    public double getX() {
        return robotX;
    }

    public double getY() {
        return robotY;
    }

    public double getHeading() {
        return robotHeading;
    }
}
