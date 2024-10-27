package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Autonomous.AllQuads2;

public class Drivetrain extends AllQuads2 {
    private DcMotor leftFrontDrive = null;  //  Used to control the left front drive wheel
    private DcMotor rightFrontDrive = null;  //  Used to control the right front drive wheel
    private DcMotor leftBackDrive = null;  //  Used to control the left back drive wheel
    private DcMotor rightBackDrive = null;  //  Used to control the right back drive wheel
    public void InitDrivetrain() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "Left_front");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "Right_front");
        leftBackDrive = hardwareMap.get(DcMotor.class, "Left_rear");
        rightBackDrive = hardwareMap.get(DcMotor.class, "Right_rear");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

     public void SetMode(DcMotor.RunMode Mode) {
        leftFrontDrive.setMode(Mode);
        rightFrontDrive.setMode(Mode);
        leftBackDrive.setMode(Mode);
        rightBackDrive.setMode(Mode);
    }
    public void SetPower(double power) {
        leftFrontDrive.setPower(power);
        rightFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
        rightBackDrive.setPower(power);
    }
    public void RecordPosition() {
        telemetry.addData("LeftFrontDrive Position", leftFrontDrive.getCurrentPosition());
        telemetry.addData("RightFrontDrive Position", rightFrontDrive.getCurrentPosition());
        telemetry.addData("LeftBackDrive Position", leftBackDrive.getCurrentPosition());
        telemetry.addData("RightBackDrive Position", rightBackDrive.getCurrentPosition());
    }
    public boolean IsAnyMotorBusy() {
        return (leftFrontDrive.isBusy() || rightFrontDrive.isBusy() || leftBackDrive.isBusy() || rightBackDrive.isBusy());
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
        SetPower(0);

    }
    public void moveAprilRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower    =  x -y -yaw;
        double rightFrontPower   =  x +y +yaw;
        double leftBackPower     =  x +y -yaw;
        double rightBackPower    =  x -y +yaw;

        // Normalize wheel powers to be less than 1.0
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
    }

    public void parkRobot(int quadrant, int Position, boolean touchPressed){
        double[] distToStrafeAtPark ={0, -33, -26, -20}; // this sets the distance once we have placed the pixel on the backboard to strafe 0 is place holder
        if (touchPressed){ //Go to the center
            if (quadrant==1|| quadrant==2){
                strafeRight(distToStrafeAtPark[Position]);
                telemetry.addData("Touch Sensor", "Is Pressed");
            } else {
                strafeLeft(distToStrafeAtPark[(Position==1)?3:(Position==3)?1:Position]);
                telemetry.addData("Touch Sensor", "Is Not Pressed");
            }
        } else {
            if (quadrant==1|| quadrant==2){
                strafeLeft(distToStrafeAtPark[(Position==1)?3:(Position==3)?1:Position]);                telemetry.addData("Touch Sensor", "Is Pressed");
                telemetry.addData("Touch Sensor", "Is Pressed");
            } else {
                strafeRight(distToStrafeAtPark[Position]);
                telemetry.addData("Touch Sensor", "Is Not Pressed");
            }
        }

    }

    public void strafeRight(double distance) {
        int ticksToMove;
        double ticksPerInch = (537.7 / ((96 / 25.4) * Math.PI)); //537.7 encoder ticks, wheel is 96mm converting to inches divide by 25.4, per one circumference of the wheel
        ticksToMove = (int) (distance * ticksPerInch);
        // Positive distance means strafe right, negative means strafe left
        leftFrontDrive.setTargetPosition(leftFrontDrive.getCurrentPosition() + (int) (ticksToMove * (distance > 0 ? 1 : -1)));
        rightFrontDrive.setTargetPosition(rightFrontDrive.getCurrentPosition() + (int) (ticksToMove * (distance > 0 ? -1 : 1)));
        leftBackDrive.setTargetPosition(leftBackDrive.getCurrentPosition() + (int) (ticksToMove * (distance > 0 ? -1 : 1)));
        rightBackDrive.setTargetPosition(rightBackDrive.getCurrentPosition() + (int) (ticksToMove * (distance > 0 ? 1 : -1)));
        double strafePwr = Range.clip((leftBackDrive.getTargetPosition()-leftFrontDrive.getCurrentPosition()) * SPEED_GAIN-0.2, -0.5 - MAX_AUTO_SPEED, 0.5+MAX_AUTO_SPEED);

        // Set the mode to RUN_TO_POSITION
        SetMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Apply power
        SetPower(strafePwr);

        // Wait for the motors to finish
        while (opModeIsActive() && IsAnyMotorBusy()) {
            //increase the speed as the distance is greater or slow down when you get getting closer
            strafePwr = Range.clip((leftBackDrive.getTargetPosition()-leftFrontDrive.getCurrentPosition()) * SPEED_GAIN-0.2, -0.5 - MAX_AUTO_SPEED, 0.5+MAX_AUTO_SPEED);
            SetPower(strafePwr);
            if (lookForRobot) {
                while (sensorRight.getDistance(DistanceUnit.CM) < DETECTION_THRESHOLD2) {
                    SetPower(0);
                    // Optionally add a sleep here to avoid a busy loop
                    sleep(100);
                    lookForRobot = false;
                }
            }
            // Optionally provide telemetry updates
            RecordPosition();
            telemetry.update();
        }

        // Stop all the motors
        SetPower(0);

        // Reset the motor modes back to RUN_USING_ENCODER
        SetMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void strafeLeft(double distance) {
        int ticksToMove;
        double ticksPerInch = (537.7 / ((96/25.4) * Math.PI));
        double strafePwr = Range.clip((leftBackDrive.getTargetPosition()-leftFrontDrive.getCurrentPosition()) * SPEED_GAIN-0.2, -0.5 - MAX_AUTO_SPEED, 0.5+MAX_AUTO_SPEED);

        ticksToMove = (int) (distance * ticksPerInch);
        // Positive distance means strafe left, negative means strafe right
        leftFrontDrive.setTargetPosition(leftFrontDrive.getCurrentPosition() + (int) (ticksToMove * (distance > 0 ? -1 : 1)));
        rightFrontDrive.setTargetPosition(rightFrontDrive.getCurrentPosition() + (int) (ticksToMove * (distance > 0 ? 1 : -1)));
        leftBackDrive.setTargetPosition(leftBackDrive.getCurrentPosition() + (int) (ticksToMove * (distance > 0 ? 1 : -1)));
        rightBackDrive.setTargetPosition(rightBackDrive.getCurrentPosition() + (int) (ticksToMove * (distance > 0 ? -1 : 1)));


        // Set the mode to RUN_TO_POSITION
        SetMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Apply power
        SetPower(strafePwr);

        // Wait for the motors to finish
        while (opModeIsActive() && IsAnyMotorBusy()) {
            //increase the speed as the distance is greater or slow down when you get getting closer
            strafePwr = Range.clip((leftBackDrive.getTargetPosition()-leftFrontDrive.getCurrentPosition()) * SPEED_GAIN-0.2, -0.5 - MAX_AUTO_SPEED, 0.5+MAX_AUTO_SPEED);
            SetPower(strafePwr);
            if (lookForRobot) {
                while (sensorLeft.getDistance(DistanceUnit.CM) < DETECTION_THRESHOLD2) {
                    SetPower(0);
                    // Optionally add a sleep here to avoid a busy loop
                    sleep(100);
                    lookForRobot = false;
                }
            }

            // Optionally provide telemetry updates
            RecordPosition();
            telemetry.update();
        }

        // Stop all the motors
        SetPower(0);

        // Reset the motor modes back to RUN_USING_ENCODER
        SetMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void driveBackward(double distance) {driveForward(-distance);}

    public void driveForward(double distance) {
        int ticksToMove;
        double ticksPerInch = (537.7 / ((96 / 25.4) * Math.PI)); //537.7 encoder ticks, wheel is 96mm converting to inches divide by 25.4, per one circumference of the wheel
        ticksToMove = (int) (distance * ticksPerInch);
        // Set the target positions for each motor
        leftFrontDrive.setTargetPosition(leftFrontDrive.getCurrentPosition() + ticksToMove);
        rightFrontDrive.setTargetPosition(rightFrontDrive.getCurrentPosition() + ticksToMove);
        leftBackDrive.setTargetPosition(leftBackDrive.getCurrentPosition() + ticksToMove);
        rightBackDrive.setTargetPosition(rightBackDrive.getCurrentPosition() + ticksToMove);
        // Set the mode to RUN_TO_POSITION
        SetMode(DcMotor.RunMode.RUN_TO_POSITION);
        double power = Range.clip((leftBackDrive.getTargetPosition()-leftFrontDrive.getCurrentPosition()) * SPEED_GAIN+0.3 * (distance > 0 ? 1 : -1), -0.5 - MAX_AUTO_SPEED, 0.5+MAX_AUTO_SPEED);
        // Apply power
        SetPower(power);

        while (opModeIsActive() && IsAnyMotorBusy()) {
            power = Range.clip((leftBackDrive.getTargetPosition()-leftFrontDrive.getCurrentPosition()) * SPEED_GAIN+0.3 * (distance > 0 ? 1 : -1), -0.5 - MAX_AUTO_SPEED, 0.5+MAX_AUTO_SPEED);
            SetPower(power);
            // Optionally provide telemetry updates
            RecordPosition();
            telemetry.update();
            if (lookForProp) {
                if (sensorLeft.getDistance(DistanceUnit.CM) < DETECTION_THRESHOLD) {
                    objectDetectedLeft = true;
                    break;  // Exit the loop if we detect an object
                }
                if (sensorRight.getDistance(DistanceUnit.CM) < DETECTION_THRESHOLD) {
                    objectDetectedRight = true;
                    break;  // Exit the loop if we detect an object
                }
            }
        }
        // Stop all the motors
        SetPower(0);
        // Reset the motor modes back to RUN_USING_ENCODER
        SetMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
