package org.firstinspires.ftc.teamcode.Components;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.Autonomous.AllQuads2;

public class QuadrantPot extends AllQuads2 {
    AnalogInput potentiometer;
    public void InitQuadrantPot() {
        potentiometer = hardwareMap.get(AnalogInput.class, "potentiometer");
    }
    public int getPotentiometerQuadrant() {
        double voltage = potentiometer.getVoltage();

        if (voltage < 0.7) {
            return 2; // This is the back of the blue alliance
        } else if (voltage >= 0.8 && voltage <= 1.2) {
            return 1; // This is the front of the blue alliance
        } else if (voltage >= 1.4 && voltage <= 2.2) {
            return 4; // This is the front of the red alliance
        } else if (voltage > 2.4) {
            return 3; // This is the back of the red alliance
        } else {
            return 0; // Return 0 if the voltage doesn't match any quadrant, this should only happen if the dial is placed on the black line
        }
    }
}
