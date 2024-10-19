package org.firstinspires.ftc.teamcode.Helper;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Disabled

public class Viper {

    double ticksPerViperInch = (537.7 / (112/25.4));
    boolean Run = true;
    public DcMotor viper;
    int currentDistance = 0;
    public DigitalChannel limit;

    public Viper(HardwareMap hardwareMap){
        viper = hardwareMap.get(DcMotor.class, "viper");

    }
    public double viperMovement(int distance) {


        //double currentDistance = viper.getCurrentPosition()/ticksPerViperInch;
        while (Run = true) {
          //  if (limit.getState() == false) {    might have to move to above viper.setTargetPosition
                int movement = (int) (ticksPerViperInch * (distance - currentDistance));
         //   if (limit.getState() == false) {
                viper.setTargetPosition(-movement);
                viper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ViperPowerCalc(distance);
                distance = currentDistance;
                break;
         //   }
          /*  else if (limit.getState() ==  true) {
                viper.setTargetPosition(0);
                viper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ViperPowerCalc(distance);
                distance = currentDistance;
                break;
                }*/

        }
        return currentDistance;
    }
    public double viperMovementSetSpeed(int distance, double power) {

        //double currentDistance = viper.getCurrentPosition()/ticksPerViperInch;
        while (Run = true) {

            int movement = (int) (ticksPerViperInch * (distance - currentDistance));
            viper.setTargetPosition(-movement);
            viper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            viper.setPower(power);
            distance = currentDistance;
            break;
        }
        return currentDistance;
    }
    public void ViperPowerCalc(int CalcDistance) {
        double viperMult = 0.01; // This is a proportionality constant. You may need to tune this value.
        while (Math.abs(getDistance() - CalcDistance) > 0.5) { // Allow for a small error margin

            double error = CalcDistance - getDistance(); // Calculate the error

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
}
