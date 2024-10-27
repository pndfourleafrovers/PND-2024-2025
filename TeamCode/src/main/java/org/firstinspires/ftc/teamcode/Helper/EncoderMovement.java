package org.firstinspires.ftc.teamcode.Helper;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.concurrent.TimeUnit;

public class EncoderMovement extends LinearOpMode {

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother jcontrol, or larger for a more aggressive response.
    final double SPEED_GAIN = 0.02;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN = 0.015;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN = 0.01;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)
    private DcMotor leftFrontDrive;  //  Used to control the left front drive wheel
    private DcMotor rightFrontDrive;  //  Used to control the right front drive wheel
    private DcMotor leftBackDrive;  //  Used to control the left back drive wheel
    private DcMotor rightBackDrive;  //  Used to control the right back drive wheel
    private DistanceSensor sensorLeft;
    private DistanceSensor sensorRight;
    private static final double DETECTION_THRESHOLD = 20.0;  // e.g., 20 cm
    private static final double DETECTION_THRESHOLD2 = 12.7;  // e.g., 20 cm
    boolean objectDetectedLeft = false;
    boolean objectDetectedRight = false;
    double fwdPwr = 0.6;
    double bwdPwr = 0.6;
    double strafePwr= 0.6;
    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private double drive = 0;
    private double strafe = 0;
    private double turn = 0;
    int Position;
    int count = 0;

    static final int TICKS_PER_MOTOR_REV = 1425;
    static final int TICKS_PER_DEGREE = TICKS_PER_MOTOR_REV / 120;
    final double TICKS_PER_DEGREES = 12;  // Adjust this value as necessary

    RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;
    RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;
    RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

    private IMU imu;      // Control/Expansion Hub IMU
    AnalogInput potentiometer;
    //This part is to create a state machine that keeps track of the state of the robot and the time at which actions are taken.

    DcMotor arm;

    enum State {
        MOVING,
        WAITING,
        READY_TO_MOVE
    }

    State state = State.READY_TO_MOVE;
    ElapsedTime timer = new ElapsedTime();
    final double WAIT_TIME = 1.0;       // Time to wait in seconds
    int moveArmToPosition = 7;          // Position to move the arm to in degrees
    public void runOpMode() {

    }
    public EncoderMovement(HardwareMap hardwareMap) {

    }
    private void initHardware(HardwareMap hardwareMap) {
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must match the names assigned during the robot configuration.
        // step (using the FTC Robot Controller app on the phone).
        leftFrontDrive = hardwareMap.get(DcMotor.class, "Left_front");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "Right_front");
        leftBackDrive = hardwareMap.get(DcMotor.class, "Left_rear");
        rightBackDrive = hardwareMap.get(DcMotor.class, "Right_rear");
        sensorLeft = hardwareMap.get(DistanceSensor.class, "LeftD"); // Replace 'sensorLeftName' with the name you've given the sensor in the configuration.
        sensorRight = hardwareMap.get(DistanceSensor.class, "RightD"); // Replace 'sensorLeftName' with the name you've given the sensor in the configuration.


        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void driveBackward(double distance, double power) {
        driveForward(-distance, power);
    }
    public void driveForward(double distance, double power) {
        int ticksToMove;
        double ticksPerInch = (537.7 / ((96/25.4) * Math.PI)); //537.7 encoder ticks, wheel is 96mm converting to inches divide by 25.4, per one circumference of the wheel
        ticksToMove = (int) (distance * ticksPerInch);
        // Set the target positions for each motor
        leftFrontDrive.setTargetPosition(leftFrontDrive.getCurrentPosition() + ticksToMove);
        rightFrontDrive.setTargetPosition(rightFrontDrive.getCurrentPosition() + ticksToMove);
        leftBackDrive.setTargetPosition(leftBackDrive.getCurrentPosition() + ticksToMove);
        rightBackDrive.setTargetPosition(rightBackDrive.getCurrentPosition() + ticksToMove);

        // Set the mode to RUN_TO_POSITION
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Apply power
        leftFrontDrive.setPower(power);
        rightFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
        rightBackDrive.setPower(power);

        while (opModeIsActive() && (leftFrontDrive.isBusy() || rightFrontDrive.isBusy() || leftBackDrive.isBusy() || rightBackDrive.isBusy())) {
            // Optionally provide telemetry updates
            telemetry.addData("LeftFrontDrive Position", leftFrontDrive.getCurrentPosition());
            telemetry.addData("RightFrontDrive Position", rightFrontDrive.getCurrentPosition());
            telemetry.addData("LeftBackDrive Position", leftBackDrive.getCurrentPosition());
            telemetry.addData("RightBackDrive Position", rightBackDrive.getCurrentPosition());
            telemetry.update();

        }
        // Stop all the motors
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        // Reset the motor modes back to RUN_USING_ENCODER
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }


    public void turnToHeading(int targetHeading) {
        double turnKp = 0.01; // This is a proportionality constant. You may need to tune this value.
        while (Math.abs(getHeading() - targetHeading) > 1) { // Allow for a small error margin

            double error = targetHeading - getHeading(); // Calculate the error

            // Calculate wheel powers based on error
            double turnPower = error * turnKp;

            double leftFrontPower    =  -turnPower;
            double rightFrontPower   =  +turnPower;
            double leftBackPower     =  -turnPower;
            double rightBackPower    =  +turnPower;

            // Ensure wheel powers do not exceed 1.0
            double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));
            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            // Send powers to the wheels.
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            telemetry.addData(">", "Robot Heading = %4.0f", getHeading());
            telemetry.update();

        }
        // Stop all the motors
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);

    }


    public void strafeLeft(double distance, double power) {
        int ticksToMove;
        double ticksPerInch = (537.7 / ((96/25.4) * Math.PI));
        ticksToMove = (int) (distance * ticksPerInch);

        // Positive distance means strafe left, negative means strafe right
        leftFrontDrive.setTargetPosition(leftFrontDrive.getCurrentPosition() + (int) (ticksToMove * (distance > 0 ? -1 : 1)));
        rightFrontDrive.setTargetPosition(rightFrontDrive.getCurrentPosition() + (int) (ticksToMove * (distance > 0 ? 1 : -1)));
        leftBackDrive.setTargetPosition(leftBackDrive.getCurrentPosition() + (int) (ticksToMove * (distance > 0 ? 1 : -1)));
        rightBackDrive.setTargetPosition(rightBackDrive.getCurrentPosition() + (int) (ticksToMove * (distance > 0 ? -1 : 1)));

        // Set the mode to RUN_TO_POSITION
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Apply power
        leftFrontDrive.setPower(power);
        rightFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
        rightBackDrive.setPower(power);

        // Wait for the motors to finish
        while (opModeIsActive() && (leftFrontDrive.isBusy() || rightFrontDrive.isBusy() || leftBackDrive.isBusy() || rightBackDrive.isBusy())) {
            // Optionally provide telemetry updates
            telemetry.addData("LeftFrontDrive Position", leftFrontDrive.getCurrentPosition());
            telemetry.addData("RightFrontDrive Position", rightFrontDrive.getCurrentPosition());
            telemetry.addData("LeftBackDrive Position", leftBackDrive.getCurrentPosition());
            telemetry.addData("RightBackDrive Position", rightBackDrive.getCurrentPosition());
            telemetry.update();
        }


        // Stop all the motors
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);

        // Reset the motor modes back to RUN_USING_ENCODER
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void strafeRight(double distance, double power) {
        int ticksToMove;
        double ticksPerInch = (537.7 / ((96/25.4) * Math.PI)); //537.7 encoder ticks, wheel is 96mm converting to inches divide by 25.4, per one circumference of the wheel
        ticksToMove = (int) (distance * ticksPerInch);

        // Positive distance means strafe right, negative means strafe left
        leftFrontDrive.setTargetPosition(leftFrontDrive.getCurrentPosition() + (int) (ticksToMove * (distance > 0 ? 1 : -1)));
        rightFrontDrive.setTargetPosition(rightFrontDrive.getCurrentPosition() + (int) (ticksToMove * (distance > 0 ? -1 : 1)));
        leftBackDrive.setTargetPosition(leftBackDrive.getCurrentPosition() + (int) (ticksToMove * (distance > 0 ? -1 : 1)));
        rightBackDrive.setTargetPosition(rightBackDrive.getCurrentPosition() + (int) (ticksToMove * (distance > 0 ? 1 : -1)));

        // Set the mode to RUN_TO_POSITION
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Apply power
        leftFrontDrive.setPower(power);
        rightFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
        rightBackDrive.setPower(power);

        // Wait for the motors to finish
        while (opModeIsActive() && (leftFrontDrive.isBusy() || rightFrontDrive.isBusy() || leftBackDrive.isBusy() || rightBackDrive.isBusy())) {
            // Optionally provide telemetry updates
            telemetry.addData("LeftFrontDrive Position", leftFrontDrive.getCurrentPosition());
            telemetry.addData("RightFrontDrive Position", rightFrontDrive.getCurrentPosition());
            telemetry.addData("LeftBackDrive Position", leftBackDrive.getCurrentPosition());
            telemetry.addData("RightBackDrive Position", rightBackDrive.getCurrentPosition());
            telemetry.update();
        }

        // Stop all the motors
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);

        // Reset the motor modes back to RUN_USING_ENCODER
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void strafeRightWithObstacleDetection(double distance, double power, double detectionThreshold) {
        int ticksToMove;
        double ticksPerInch = (537.7 / ((96 / 25.4) * Math.PI));
        ticksToMove = (int) (distance * ticksPerInch);

        // Set target positions for strafing right
        leftFrontDrive.setTargetPosition(leftFrontDrive.getCurrentPosition() + ticksToMove);
        rightFrontDrive.setTargetPosition(rightFrontDrive.getCurrentPosition() - ticksToMove);
        leftBackDrive.setTargetPosition(leftBackDrive.getCurrentPosition() - ticksToMove);
        rightBackDrive.setTargetPosition(rightBackDrive.getCurrentPosition() + ticksToMove);

        // Set the mode to RUN_TO_POSITION
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Apply power
        leftFrontDrive.setPower(power);
        rightFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
        rightBackDrive.setPower(power);

        // Wait for the motors to finish or stop if an obstacle is detected
        while (opModeIsActive() && (leftFrontDrive.isBusy() || rightFrontDrive.isBusy() || leftBackDrive.isBusy() || rightBackDrive.isBusy())) {
            // Check for obstacle
            if (sensorLeft.getDistance(DistanceUnit.CM) < DETECTION_THRESHOLD2) {
                // Obstacle detected, stop all motors
                leftFrontDrive.setPower(0);
                rightFrontDrive.setPower(0);
                leftBackDrive.setPower(0);
                rightBackDrive.setPower(0);

                // Wait until the obstacle is cleared
                while (sensorLeft.getDistance(DistanceUnit.CM) < DETECTION_THRESHOLD2) {
                    // Optionally add a sleep here to avoid a busy loop
                    sleep(100);
                }

                // Once the obstacle is cleared, resume movement
                leftFrontDrive.setPower(power);
                rightFrontDrive.setPower(power);
                leftBackDrive.setPower(power);
                rightBackDrive.setPower(power);
            }

            // Optionally provide telemetry updates
            telemetry.addData("LeftFrontDrive Position", leftFrontDrive.getCurrentPosition());
            telemetry.addData("RightFrontDrive Position", rightFrontDrive.getCurrentPosition());
            telemetry.addData("LeftBackDrive Position", leftBackDrive.getCurrentPosition());
            telemetry.addData("RightBackDrive Position", rightBackDrive.getCurrentPosition());
            telemetry.update();
        }

        // Stop all the motors
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);

        // Reset the motor modes back to RUN_USING_ENCODER
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public void strafeLeftWithObstacleDetection(double distance, double power, double detectionThreshold) {
        int ticksToMove;
        double ticksPerInch = (537.7 / ((96 / 25.4) * Math.PI));
        ticksToMove = (int) (distance * ticksPerInch);

        // Set target positions for strafing left
        leftFrontDrive.setTargetPosition(leftFrontDrive.getCurrentPosition() - ticksToMove);
        rightFrontDrive.setTargetPosition(rightFrontDrive.getCurrentPosition() + ticksToMove);
        leftBackDrive.setTargetPosition(leftBackDrive.getCurrentPosition() + ticksToMove);
        rightBackDrive.setTargetPosition(rightBackDrive.getCurrentPosition() - ticksToMove);

        // Set the mode to RUN_TO_POSITION
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Apply power
        leftFrontDrive.setPower(power);
        rightFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
        rightBackDrive.setPower(power);

        // Wait for the motors to finish or stop if an obstacle is detected
        while (opModeIsActive() && (leftFrontDrive.isBusy() || rightFrontDrive.isBusy() || leftBackDrive.isBusy() || rightBackDrive.isBusy())) {
            // Check for obstacle
            if (sensorRight.getDistance(DistanceUnit.CM) < DETECTION_THRESHOLD2) {
                // Obstacle detected, stop all motors
                leftFrontDrive.setPower(0);
                rightFrontDrive.setPower(0);
                leftBackDrive.setPower(0);
                rightBackDrive.setPower(0);

                // Wait until the obstacle is cleared
                while (sensorRight.getDistance(DistanceUnit.CM) < DETECTION_THRESHOLD2) {
                    // Optionally add a sleep here to avoid a busy loop
                    sleep(100);
                }

                // Once the obstacle is cleared, resume movement
                leftFrontDrive.setPower(power);
                rightFrontDrive.setPower(power);
                leftBackDrive.setPower(power);
                rightBackDrive.setPower(power);
            }

            // Optionally provide telemetry updates
            telemetry.addData("LeftFrontDrive Position", leftFrontDrive.getCurrentPosition());
            telemetry.addData("RightFrontDrive Position", rightFrontDrive.getCurrentPosition());
            telemetry.addData("LeftBackDrive Position", leftBackDrive.getCurrentPosition());
            telemetry.addData("RightBackDrive Position", rightBackDrive.getCurrentPosition());
            telemetry.update();
        }

        // Stop all the motors
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);

        // Reset the motor modes back to RUN_USING_ENCODER
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void controlArm(int desiredPosition) {
        switch (state) {
            case READY_TO_MOVE:
                // Ready to move to a new position
                int targetPosition = (int) (TICKS_PER_DEGREES * desiredPosition);
                arm.setTargetPosition(targetPosition);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(0.5);
                state = State.MOVING;
                break;

            case MOVING:
                // Wait for the arm to reach the target position
                if (!arm.isBusy()) {
                    // Arm has reached the target, start the timer and switch to waiting state
                    timer.reset();
                    state = State.WAITING;
                }
                break;

            case WAITING:
                // Wait for the specified time
                if (timer.seconds() > WAIT_TIME) {
                    // Time is up, ready to move to a new position
                    state = State.READY_TO_MOVE;
                }
                break;
        }
    }
}
