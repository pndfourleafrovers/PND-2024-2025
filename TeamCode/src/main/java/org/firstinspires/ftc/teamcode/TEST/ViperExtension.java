package org.firstinspires.ftc.teamcode.TEST;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name="ViperExtension", group="TeleOp")
public class ViperExtension extends LinearOpMode {
    static final int TICKS_PER_MOTOR_REV = 1425;
    static final int TICKS_PER_GEAR_REV = TICKS_PER_MOTOR_REV * 3;
    static final int TICKS_PER_DEGREE = TICKS_PER_GEAR_REV / 360;

    double ticksPerGearRev = 112/25.4;
    double ticksPerViperInch = (537.7 / ((112/25.4) * Math.PI));
    boolean Run = true;
    private DcMotor viper;
    private DcMotor arm;
    int currentDegree = 0;
    int currentDistance = 0;
    @Override
    public void runOpMode() {
        viper = hardwareMap.get(DcMotor.class, "viper");
        arm = hardwareMap.get(DcMotor.class, "arm");

        telemetry.addData("viper", viper.getCurrentPosition());

    waitForStart();
    while(opModeIsActive()){


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
            armMovement(holdPosition);
        }


        if (gamepad1.right_stick_button) {
            viper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            double viperMove = -gamepad1.left_stick_y;
            telemetry.addData("Arm Position", viper.getCurrentPosition() / TICKS_PER_DEGREE);
            telemetry.update();
            double viperPowerV = viperMove;
            if (viperPowerV < -0.1) {
                viperPowerV = -0.1;
            } else if (viperPowerV > 0.2) {
                viperPowerV = 0.2;
            } else if (viperPowerV == 0) {
                viperPowerV = 0.0000001;
            }
            viper.setPower(viperPowerV);
        }
        if (gamepad1.back) {
            int holdPosition = (int)(viper.getCurrentPosition() / ticksPerViperInch);
            telemetry.addData("Viper Position", viper.getCurrentPosition() / ticksPerViperInch);
            telemetry.update();
            viperMovement(holdPosition, 0.2);
        }



        if(gamepad1.a){
            viperMovementSetSpeed(0,0.2);
        }
        if(gamepad1.b){
            viperMovementSetSpeed(10,0.5);
        }
        if(gamepad1.x){
            viperMovementSetSpeed(20,0.5);
        }
        if(gamepad1.y){
            viperMovementSetSpeed(30,0.5);
        }
        if(gamepad2.a){
            viperMovement(30,0);
        }




    }


    }
// viper is backwards for some reason. Either change mechanics or add a negative to the double current distance
    // mess around with power, changed some things, but left some leftover code in case we need to change back for testing
    private double viperMovement(int distance, double power) {


      //  int currentDistance = viper.getCurrentPosition()/ticksPerViperInch;
        while (Run = true) {

            viper.setDirection(DcMotor.Direction.REVERSE);
            int movement = (int) (ticksPerViperInch * (distance - currentDistance));
            viper.setTargetPosition(movement);
            viper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ViperPowerCalc(distance);
            distance = currentDistance;
            telemetry.addData("Viper Position:", currentDistance);
            telemetry.update();
            break;
        }
        return currentDistance;
    }
    private double viperMovementSetSpeed(int distance, double power) {


        //  int currentDistance = viper.getCurrentPosition()/ticksPerViperInch;
        while (Run = true) {

            viper.setDirection(DcMotor.Direction.REVERSE);

            int movement = (int) (ticksPerViperInch * (distance - currentDistance));
            viper.setTargetPosition(movement);
            viper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            viper.setPower(power);
            distance = currentDistance;
            telemetry.addData("Viper Position:", currentDistance);
            telemetry.update();
            break;
        }
        return currentDistance;
    }

    public void ViperPowerCalc(int CalcDistance) {
        double viperMult = 0.01; // This is a proportionality constant. You may need to tune this value.
        while (Math.abs(getDegree() - CalcDistance) > 0.5) { // Allow for a small error margin

            double error = CalcDistance - getDegree(); // Calculate the error

            // Calculate wheel powers based on error
            double viperPower = error * viperMult;
            double Low = 0.2;
            if(viperPower<Low)
                viperPower = Low;
            viper.setPower(viperPower);
        }
    }

    public double getDistance() {
        double viperPos;
        viperPos = viper.getCurrentPosition()/ticksPerViperInch;
        return viperPos;
    }






        private int armMovement(int degree) {
            //    Run = true;
            while (Run = true) {

                arm.setTargetPosition(TICKS_PER_DEGREE * (degree - currentDegree));
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ArmPowerCalc(degree);
                degree = currentDegree;
                telemetry.addData("Arm Position:", currentDegree);
                telemetry.update();
                break;
            }
            return currentDegree;
        }

        public double getDegree() {
            double armPos;
            armPos = arm.getCurrentPosition()/TICKS_PER_DEGREE;
            return armPos;
        }

    public void ArmPowerCalc(int CalcDegree) {
        double ArmMult = 0.01; // This is a proportionality constant. You may need to tune this value.
        while (Math.abs(getDegree() - CalcDegree) > 0.5) { // Allow for a small error margin

            double error = CalcDegree - getDegree(); // Calculate the error

            // Calculate wheel powers based on error
            double armPower = error * ArmMult;
            double Low = 0.2;
            if(armPower<Low)
                armPower = Low;
            arm.setPower(armPower);
        }
    }


    }



















