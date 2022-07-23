package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.hardware.Servo;

// Based on FTC external.samples.ConceptScanServo.
public class SwivelServo {

    private final Servo swivelServo;

    private static final double INCREMENT = 0.01;     // amount to slew servo each CYCLE_MS cycle
    private static final double MAX_POS = 1.0;     // Maximum rotational position
    private static final double MIN_POS = 0.0;     // Minimum rotational position

    private double position;
    private boolean rampUp = true;

    public SwivelServo(Servo pServo) {
        swivelServo = pServo;
        position = (MAX_POS - MIN_POS) / 2; // Start at halfway position
    }

    public double moveServo() {
        // slew the servo, according to the rampUp (direction) variable.
        if (rampUp) {
            // Keep stepping up until we hit the max value.
            position += INCREMENT;
            if (position >= MAX_POS) {
                position = MAX_POS;
                rampUp = false;   // Switch ramp direction
            }
        } else {
            // Keep stepping down until we hit the min value.
            position -= INCREMENT;
            if (position <= MIN_POS) {
                position = MIN_POS;
                rampUp = true;  // Switch ramp direction
            }
        }

        // Set the servo to the new position and pause;
        swivelServo.setPosition(position);
        return position;
    }
}
