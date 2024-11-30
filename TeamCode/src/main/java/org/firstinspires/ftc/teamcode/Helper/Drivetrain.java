package org.firstinspires.ftc.teamcode.Helper;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Components.SharedData;

public class Drivetrain {
    private DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive;
    private DcMotorEx leftEncoder, rightEncoder, horizontalEncoder;
    private IMU imu;
    private final Telemetry telemetry;
    private SharedData sharedData;

    // Odometry constants
    private static final double WHEEL_DIAMETER_INCHES = 38.0 / 25.4; // 38mm to inches
    private static final double TICKS_PER_REV = 2000.0;
    private static final double INCHES_PER_TICK = (Math.PI * WHEEL_DIAMETER_INCHES) / TICKS_PER_REV;
    private static final double WHEEL_BASE_WIDTH = 10.75; // in inches
    private static final double HORIZONTAL_OFFSET = 6.5;  // in inches

    // Robot state
    private double xPosition = 0.0;
    private double yPosition = 0.0;
    private double heading = 0.0;
    private double previousLeftEncoderPos = 0.0;
    private double previousRightEncoderPos = 0.0;
    private double previousHorizontalEncoderPos = 0.0;

    public Drivetrain(HardwareMap hardwareMap, Telemetry telemetry, SharedData sharedData) {
        this.telemetry = telemetry;
        this.sharedData = sharedData;

        leftFrontDrive = hardwareMap.get(DcMotor.class, "Left_front");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "Right_front");
        leftBackDrive = hardwareMap.get(DcMotor.class, "Left_rear");
        rightBackDrive = hardwareMap.get(DcMotor.class, "Right_rear");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Odometry encoders
        leftEncoder = hardwareMap.get(DcMotorEx.class, "Left_rear");
        rightEncoder = hardwareMap.get(DcMotorEx.class, "Right_rear");
        horizontalEncoder = hardwareMap.get(DcMotorEx.class, "Left_front");

        resetEncoders();

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();
    }

    private void resetEncoders() {
        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontalEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void updateOdometry() {
        // Get current positions
        double leftPosition = -1*leftEncoder.getCurrentPosition() * INCHES_PER_TICK;
        double rightPosition = -1*rightEncoder.getCurrentPosition() * INCHES_PER_TICK;
        double horizontalPosition = -1*horizontalEncoder.getCurrentPosition() * INCHES_PER_TICK;

        // Calculate changes
        double deltaLeft = leftPosition - previousLeftEncoderPos;
        double deltaRight = rightPosition - previousRightEncoderPos;
        double deltaHorizontal = horizontalPosition - previousHorizontalEncoderPos;

        // Update previous positions
        previousLeftEncoderPos = leftPosition;
        previousRightEncoderPos = rightPosition;
        previousHorizontalEncoderPos = horizontalPosition;

        // Calculate heading change
        double deltaHeading = (deltaRight - deltaLeft) / WHEEL_BASE_WIDTH;

        // Calculate position changes
        double deltaX = deltaHorizontal - HORIZONTAL_OFFSET * deltaHeading;
        double deltaY = (deltaLeft + deltaRight) / 2.0;

        // Update robot position
        heading += deltaHeading;
        xPosition += deltaX * Math.cos(heading) - deltaY * Math.sin(heading);
        yPosition += deltaX * Math.sin(heading) + deltaY * Math.cos(heading);

        telemetry.addData("Odometry X Position", xPosition);
        telemetry.addData("Odometry Y Position", yPosition);
        telemetry.addData("Odometry Heading", Math.toDegrees(heading));
        telemetry.update();
    }

    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

    public void SetPower(double power) {
        leftFrontDrive.setPower(power);
        rightFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
        rightBackDrive.setPower(power);
    }

    // Method to drive forward with adjustable power
    public void driveForward(double distanceInches, double power) {
        resetEncoders();
        double targetTicks = distanceInches / INCHES_PER_TICK;

        while (Math.abs(leftFrontDrive.getCurrentPosition()) < targetTicks) {
            updateOdometry();
            leftFrontDrive.setPower(power);
            rightFrontDrive.setPower(power);
            leftBackDrive.setPower(power);
            rightBackDrive.setPower(power);

            telemetry.addData("Driving Forward", "Position: %f / %f", Math.abs(leftFrontDrive.getCurrentPosition() * INCHES_PER_TICK), distanceInches);
            telemetry.update();
        }
        SetPower(0); // Stop motors after reaching target
    }

    // Method to strafe left with adjustable power
    public void strafeLeft(double distanceInches, double power) {
        resetEncoders();
        double targetTicks = distanceInches / INCHES_PER_TICK;

        while (Math.abs(leftFrontDrive.getCurrentPosition()) < targetTicks) {
            updateOdometry();
            leftFrontDrive.setPower(-power);
            rightFrontDrive.setPower(power);
            leftBackDrive.setPower(power);
            rightBackDrive.setPower(-power);

            telemetry.addData("Strafing Left", "Position: %f / %f", Math.abs(leftFrontDrive.getCurrentPosition() * INCHES_PER_TICK), distanceInches);
            telemetry.update();
        }
        SetPower(0); // Stop motors after reaching target
    }

    // Method to turn to a specified heading with adjustable power
    public void turnToHeading(double targetHeadingDegrees, double power) {
        double targetHeading = Math.toRadians(targetHeadingDegrees);
        resetEncoders();

        while (Math.abs(targetHeading - heading) > Math.toRadians(1.0)) {
            updateOdometry();
            double turnPower = Range.clip(targetHeading - heading, -power, power);

            leftFrontDrive.setPower(-turnPower);
            rightFrontDrive.setPower(turnPower);
            leftBackDrive.setPower(-turnPower);
            rightBackDrive.setPower(turnPower);

            telemetry.addData("Turning to Heading", "Heading: %f / %f", Math.toDegrees(heading), targetHeadingDegrees);
            telemetry.update();
        }
        SetPower(0); // Stop motors after reaching target heading
    }
}
