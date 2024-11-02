package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Helper.Arm;
import org.firstinspires.ftc.teamcode.Helper.SMM;
import org.firstinspires.ftc.teamcode.Helper.Viper;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp(name="BasicMovement", group="TeleOp")
//@Disabled
public class BasicMovement extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN = 0.02;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN = 0.015;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN = 0.01;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE = 0.5;   //  Clip the approach speed to this max value (adjust for yougfmr robot)
    final double MAX_AUTO_TURN = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FrontLeft;
    private DcMotor RearLeft;
    private boolean slowMode = false;
    private double powerMultiplier = 1.0;
    private DcMotor FrontRight;
    private DcMotor RearRight;
    private static final boolean USE_WEBCAM = true;
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection detectedTag = null;     // Used to hold the data for a detected AprilTag
    private double drive = 0;
    private double strafe = 0;
    private double turn = 0;
    private Servo pmmA;
    private Servo drone;
    private DcMotor arm;
    int currentDegree = 0;
    static final int TICKS_PER_MOTOR_REV = 1425;
    static final int TICKS_PER_GEAR_REV = TICKS_PER_MOTOR_REV * 3;
    static final int TICKS_PER_DEGREE = TICKS_PER_GEAR_REV / 360;   //  /120;
    int armPosition = 819;
    private Servo pmmF;
    boolean APRIL = true;
    boolean Run = true;
    private IMU imu = null;
    RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;
    RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;
    RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
    private Viper Viper;
    private SMM SMM;
    private Arm Arm;
    double ticksPerViperInch = (537.7 / (112/25.4));

    @Override
    public void runOpMode() {
        Viper = new Viper(hardwareMap);
        SMM = new SMM(hardwareMap);
        Arm = new Arm(hardwareMap);
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        FrontLeft = hardwareMap.get(DcMotor.class, "Left_front");
        RearLeft = hardwareMap.get(DcMotor.class, "Left_rear");
        FrontRight = hardwareMap.get(DcMotor.class, "Right_front");
        RearRight = hardwareMap.get(DcMotor.class, "Right_rear");

        arm = hardwareMap.get(DcMotor.class, "arm");


        FrontLeft.setDirection(DcMotor.Direction.REVERSE);
        RearLeft.setDirection(DcMotor.Direction.REVERSE);
        FrontRight.setDirection(DcMotor.Direction.FORWARD);
        RearRight.setDirection(DcMotor.Direction.FORWARD);


        initHardware();


        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        telemetry.addData("Arm Position", arm.getCurrentPosition() / TICKS_PER_DEGREE);
        telemetry.addData("Viper Position", Viper.viper.getCurrentPosition() / ticksPerViperInch);

        //  telemetry.addData(">", "Robot Heading = %4.0f", getHeading());
        telemetry.update();
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Arm Position", arm.getCurrentPosition() / TICKS_PER_DEGREE);
            telemetry.addData("Viper Position", Viper.viper.getCurrentPosition() / ticksPerViperInch);

            //  telemetry.addData(">", "Robot Heading = %4.0f", getHeading());
            telemetry.update();
            double max;


            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));
            //    max = Math.max(max, Math.abs(armPower));

            if (gamepad1.right_bumper) {
                if (!slowMode) {
                    slowMode = true;
                    powerMultiplier = 0.2;
                }
            } else {
                if (slowMode) {
                    slowMode = false;
                    powerMultiplier = 1.0;
                }
            }

            FrontLeft.setPower(leftFrontPower * powerMultiplier);
            FrontRight.setPower(rightFrontPower * powerMultiplier);
            RearLeft.setPower(leftBackPower * powerMultiplier);
            RearRight.setPower(rightBackPower * powerMultiplier);

            if (gamepad2.right_stick_button) {
                arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                double armMove = -gamepad2.left_stick_y;
                telemetry.addData("Arm Position", arm.getCurrentPosition() / TICKS_PER_DEGREE);
                telemetry.update();
                double armPowerV = armMove;
                if (armPowerV < -0.1) {
                    armPowerV = -0.1;
                } else if (armPowerV > 0.2) {
                    armPowerV = 0.2;
                } else if (armPowerV == 0) {
                    armPowerV = 0.0000001;
                }
                arm.setPower(armPowerV);
            }
            if (gamepad2.back) {
                int holdPosition = arm.getCurrentPosition() / TICKS_PER_DEGREE;
                telemetry.addData("Arm Position", arm.getCurrentPosition() / TICKS_PER_DEGREE);
                telemetry.update();
                Arm.armMovement(holdPosition);
            }

/*
            if (gamepad2.a) {
                Viper.viper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                double viperMove = -gamepad2.left_stick_y;
                telemetry.addData("Arm Position", Viper.viper.getCurrentPosition() / TICKS_PER_DEGREE);
                telemetry.update();
                double viperPowerV = viperMove;
                if (viperPowerV < -0.1) {
                    viperPowerV = -0.1;
                } else if (viperPowerV > 0.2) {
                    viperPowerV = 0.2;
                } else if (viperPowerV == 0) {
                    viperPowerV = 0.0000001;
                }
                Viper.viper.setPower(viperPowerV);
            }
            if (gamepad2.back) {n() / ticksPerViperInch);
                telemetry.addData("Viper Position", Viper.viper.getCurrentPosition() / ticksPerViperInch);
                telemetry.update();
                Viper.viperMovementSetSpeed(holdPosition, 0.2);
                int holdPosition = (int) (Viper.viper.getCurrentPositio
            }
*/
            // viper movement

            //top basket
            if (gamepad2.a) {
                Arm.armMovement(92);
                Viper.viperMovementSetSpeed(24, 1);
            }

            if (gamepad2.x) {
                Viper.viperMovementSetSpeed(0, 1);
                sleep(2000);
                Arm.armMovementSetPower(0,1);

            }

            //rest to 0
            if (gamepad2.b) {
                Viper.viperMovementSetSpeed(0, 1);

            }

            // inner pos 1
            if (gamepad2.dpad_left) {
                Viper.viperMovementSetSpeed(6, 0.3);
                Arm.armMovement(-2);
            }

            // inner pos 2
                if (gamepad2.dpad_up) {
                    Arm.armMovementSetPower(4,1);

                    Viper.viperMovementSetSpeed(10, 0.3);
                    Arm.armMovementSetPower(2,0.7);
                 //   Viper.viperMovementSetSpeed(10, 0.5);
                }

            // inner pos 3
            if (gamepad2.dpad_right) {
                Arm.armMovementSetPower(4,1);
                Viper.viperMovementSetSpeed(12, 0.3);
                Arm.armMovementSetPower(7,0.7);
              //  Viper.viperMovementSetSpeed(12, 0.5);
            }

            if (gamepad2.dpad_down){
                Viper.viperMovementSetSpeed(15, 0.5);
                Arm.armMovementSetPower(9, 1);

            }
            if (gamepad2.y) {
                Arm.armMovementSetPower(20, 1);
                Viper.viperMovementSetSpeed(0, 0.5);

            }

                //backward
                if (gamepad1.dpad_down) {
                    SMM.smmWMovement(-1);
                }
                //forward
                else if (gamepad1.dpad_up) {
                    SMM.smmWMovement(1);
                // no power
                } else {
                    SMM.smmWMovement(0);
                }

                //pos 1 smm
                if (gamepad2.right_bumper) {
                    SMM.smmTMovement(1);
                }
                //pos 2 smm
                else {
                    SMM.smmTMovement(0);
                }


                // level 2 preparing position
                if (gamepad1.a) {
                    Arm.armMovement(152);
                }

            // ground holding position
            if (gamepad1.left_bumper) {
                Arm.armMovementSetPower(7,1);
            }
            }
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
    }
    }
