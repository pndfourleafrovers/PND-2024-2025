package org.firstinspires.ftc.teamcode.TEST;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous (name = "Odometry", group = "Autonomous")
@Disabled

public class Odometry_Basic extends LinearOpMode {
    private DcMotor leftFrontDrive = null;  //  Used to control the left front drive wheel
    private DcMotor rightFrontDrive = null;  //  Used to control the right front drive wheel
    private DcMotor leftBackDrive = null;  //  Used to control the left back drive wheel
    private DcMotor rightBackDrive = null;  //  Used to control the right back drive wheel
    private DcMotorEx centerEncoder, rightEncoder, leftEncoder;
    double ticksPerWheelInch = (537.7 / ((96/25.4) * Math.PI)); //537.7 encoder ticks, wheel is 96mm converting to inches divide by 25.4, per one circumference of the wheel
    double ticksPerEncoderInch = (2000/ ((48/25.4)*Math.PI)); // Dead wheels
    double ticksPerEncoderPerTicksPerWheel = ticksPerEncoderInch/ticksPerWheelInch;
    double trackwidth = 0;
    double trackheight = 0;
 //   private final HolonomicIMUOdometry odometry;
    int currentRightPosition = 0;
    int currentLeftPosition = 0;
    int currentCenterPosition = 0;
    int pos_x = 0;
    int pos_y = 0;
    int pos_del = 0;

    @Override
    public void runOpMode() {

        initHardware();

        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Center Encoder", centerEncoder.getCurrentPosition());
            telemetry.addData("RightEncoder Position", rightEncoder.getCurrentPosition());
            telemetry.addData("LeftEncoder Position", leftEncoder.getCurrentPosition());
            telemetry.update();

        OdoMovement(7,0.1);



        }








    }

/*
    public void updatePose(double heading){
        odometry.updatePose(heading);
    }
*/

public void OdoMovement(double distance, double power) {
    //This method tracks the changes in position while moving.
    int oldRightPosition;
    int oldLeftPosition;
    int oldCenterPosition;
    int ticksToMoveEncoder = (int)(distance *ticksPerEncoderInch);

    /**
     * Movement probably doesn't work and, if it does, is not at the actually good level yet.
     */
    centerEncoder.setTargetPosition(centerEncoder.getCurrentPosition() + ticksToMoveEncoder);
    rightEncoder.setTargetPosition(rightEncoder.getCurrentPosition() + ticksToMoveEncoder);
    leftEncoder.setTargetPosition(leftEncoder.getCurrentPosition() + ticksToMoveEncoder);
    // Set the mode to RUN_TO_POSITION
    centerEncoder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    rightEncoder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    leftEncoder.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    // Apply power
    leftFrontDrive.setPower(power);
    rightFrontDrive.setPower(power);
    leftBackDrive.setPower(power);
    rightBackDrive.setPower(power);



    /**
     * Below is the math!
     */
    oldRightPosition = currentRightPosition;
    oldLeftPosition = currentLeftPosition;
    oldCenterPosition = currentCenterPosition;

    currentRightPosition = -rightEncoder.getCurrentPosition();
    currentLeftPosition = leftEncoder.getCurrentPosition();
    currentCenterPosition = centerEncoder.getCurrentPosition();



    int dn1 = currentLeftPosition  - oldLeftPosition;
    int dn2 = currentRightPosition - oldRightPosition;
    int dn3 = currentCenterPosition - oldCenterPosition;






    // the robot has moved and turned a tiny bit between two measurements:
    double delTheta = ticksPerEncoderInch * ((dn2-dn1) / (trackwidth));
    double delx = ticksPerEncoderInch * ((dn1+dn2) / 2.0);
    double dely = ticksPerEncoderInch * (dn3 + ((dn2-dn1) / 2.0) * trackheight);

    // small movement of the robot gets added to the field coordinate system:
    pos_del += delTheta / 2;

    pos_x += delx * Math.cos(pos_del) - dely * Math.sin(pos_del);
    pos_y += delx * Math.sin(pos_del) + dely * Math.cos(pos_del);
    pos_del += delTheta / 2;
    pos_del += delTheta;

    centerEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    rightEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    leftEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
}
/**
 *
 here for reference
 */
    public void driveForward(double distance, double power) {
      //  int ticksToMoveWheel;
        int ticksToMoveEncoder;
     //   ticksToMoveWheel = (int) (distance * ticksPerWheelInch);
        ticksToMoveEncoder = (int)(distance *ticksPerEncoderInch);
        // Set the target positions for each motor
      //  leftFrontDrive.setTargetPosition(leftFrontDrive.getCurrentPosition() + ticksToMoveWheel);
      //  rightFrontDrive.setTargetPosition(rightFrontDrive.getCurrentPosition() + ticksToMoveWheel);
      //  leftBackDrive.setTargetPosition(leftBackDrive.getCurrentPosition() + ticksToMoveWheel);
      //  rightBackDrive.setTargetPosition(rightBackDrive.getCurrentPosition() + ticksToMoveWheel);

        centerEncoder.setTargetPosition(centerEncoder.getCurrentPosition() + ticksToMoveEncoder);
        rightEncoder.setTargetPosition(rightEncoder.getCurrentPosition() + ticksToMoveEncoder);
        leftEncoder.setTargetPosition(leftEncoder.getCurrentPosition() + ticksToMoveEncoder);
        // Set the mode to RUN_TO_POSITION
        centerEncoder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightEncoder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftEncoder.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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
            telemetry.addData("CenterEncoder Position", centerEncoder.getCurrentPosition());
            telemetry.addData("RightEncoder Position", rightEncoder.getCurrentPosition());
            telemetry.addData("LeftEncoder Position", leftEncoder.getCurrentPosition());
            telemetry.update();

        }
        // Stop all the motors
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);


    }

    private void pose(int x,int y,int h){

    }


    private void initHardware() {
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must match the names assigned during the robot configuration.
        // step (using the FTC Robot Controller app on the phone).
        leftFrontDrive = hardwareMap.get(DcMotor.class, "Left_front");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "Right_front");
        leftBackDrive = hardwareMap.get(DcMotor.class, "Left_rear");
        rightBackDrive = hardwareMap.get(DcMotor.class, "Right_rear");

        centerEncoder = hardwareMap.get(DcMotorEx.class, "Center_Encoder");
        rightEncoder = hardwareMap.get(DcMotorEx.class, "Right_Encoder");
        leftEncoder = hardwareMap.get(DcMotorEx.class, "Left_Encoder");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

      /**  leftEncoder = motor;
        rightEncoder = motor;        // shadow nearest real motor
        centerEncoder = motor;
        */
        reset_encoders();
    }

public void reset_encoders(){
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }





}
