package frc.robot.utils;

import edu.wpi.first.wpilibj.Timer;

public class TimingUtils {

    private double time;

    /**
     * Timing related utilities
     */
    public TimingUtils () {}

    /**
     * Reset the timer
     */
    public void reset() {
        this.time = Timer.getFPGATimestamp() / 1000;
    }

    /**
     * Run a snippet of code every [mills] ms (should be called in periodic)
     * @param toRun The snippet of code to run
     * @param mills Amount of time between each execute
     */
    public void runEvery(Runnable toRun, double mills) {
        if ((Timer.getFPGATimestamp() / 1000) - this.time >= mills) {
            this.time = Timer.getFPGATimestamp() / 1000;
            toRun.run();
        }
    }
}
