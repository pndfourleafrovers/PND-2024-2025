package org.firstinspires.ftc.teamcode.Helper;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Disabled

public class Arm {
    static final int TICKS_PER_MOTOR_REV = 1425;
    static final int TICKS_PER_GEAR_REV = TICKS_PER_MOTOR_REV * 3;
    static final int TICKS_PER_DEGREE = TICKS_PER_GEAR_REV / 360;

    private DcMotor arm;
    int currentDegree = 0;
    boolean Run = true;
    private DcMotor FrontRight;
    private DcMotor RearRight;
    private DcMotor FrontLeft;
    private DcMotor RearLeft;
    public Arm(HardwareMap hardwareMap){
        arm = hardwareMap.get(DcMotor.class, "arm");

    }
    public int armMovement(int degree) {
        //    Run = true;
        while (Run = true) {

            arm.setTargetPosition(TICKS_PER_DEGREE * (degree - currentDegree));
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ArmPowerCalc(degree);
            degree = currentDegree;
            break;
        }
        return currentDegree;
    }
    public int armMovementSetPower(int degree, double power) {
        //    Run = true;
        while (Run = true) {

            arm.setTargetPosition(TICKS_PER_DEGREE * (degree - currentDegree));
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(power);
            degree = currentDegree;
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
