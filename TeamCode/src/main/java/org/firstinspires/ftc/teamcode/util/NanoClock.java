package org.firstinspires.ftc.teamcode.util;


public abstract class NanoClock {
    public static final NanoClock.Companion Companion = new Companion();

    public abstract double seconds();

    public static final NanoClock system() {
        return Companion.system();
    }

    public static final class Companion {

        public final NanoClock system() {
            return (NanoClock)(new NanoClock() {
                public double seconds() {
                    return (double)System.nanoTime() / 1.0E9D;
                }
            });
        }
    }
}