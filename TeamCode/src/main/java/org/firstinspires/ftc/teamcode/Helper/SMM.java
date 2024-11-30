package org.firstinspires.ftc.teamcode.Helper;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImpl;

//smm stands for sample manipulating mechanism
@Disabled

public class SMM {
    private CRServoImpl smmW;
    private Servo smmT;
    public Servo smmG;

    public SMM(HardwareMap hardwareMap){
        // smmW = hardwareMap.get(CRServoImpl.class, "smmW");
        smmT = hardwareMap.get(Servo.class, "smmT");
        smmG = hardwareMap.get(Servo.class, "smmG");

    }
    //turns wheels to take in samples, servo called smmW
    public void smmWMovement(double power) {
        smmW.setPower(power);
    }

    //for turning the smm overall, motor called smmT
    public void smmTMovement(double state) {
        smmT.setDirection(Servo.Direction.FORWARD);
        smmT.setPosition(state);


    }
    public void smmGMovement(double state){
        smmG.setDirection(Servo.Direction.REVERSE);
        smmG.setPosition(state);
    }
}