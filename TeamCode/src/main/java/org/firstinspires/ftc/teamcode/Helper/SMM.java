package org.firstinspires.ftc.teamcode.Helper;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
//smm stands for sample manipulating mechanism
@Disabled

public class SMM {
    private CRServoImpl smmW;
    private Servo smmT;
public SMM(HardwareMap hardwareMap){
    smmW = hardwareMap.get(CRServoImpl.class, "smmW");
    smmT = hardwareMap.get(Servo.class, "smmT");
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
}


















