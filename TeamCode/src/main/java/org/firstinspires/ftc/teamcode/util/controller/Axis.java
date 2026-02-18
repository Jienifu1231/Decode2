package org.firstinspires.ftc.teamcode.util.controller;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Locale;

/**
 * A Controller Axis
 */
public class Axis{
    private final ElapsedTime timer;
    private double value;
    private double threshold = 0.05;

    /**
     * Instantiates the Button
     */
    public Axis() {
        timer = new ElapsedTime();
    }

    /**
     * The Axis's current value
     *
     * @return the current value
     */
    public double value() {
        return value;
    }

    /**
     * Whether the value has been zero long enough
     *
     * @return whether the timer is under the wait time
     */
    public boolean hasBeenZero() {
        return timer.time() >= 0.5;
    }

    public boolean moved(){ return Math.abs(value) > threshold; }

    /**
     * The Axis as a String to three decimal places
     *
     * @return whether the Axis is pressed
     */
    @NonNull
    public String toString() {
        return String.format(Locale.US, "%.3f", value);
    }

    /**
     * Updates the Axis
     *
     * @param newValue the new value
     */
    public void update(double newValue) {
        if(Math.abs(newValue) >= 0.01) {
            value = newValue;
            timer.reset();
        }
        else
            value = 0.0;
    }
}