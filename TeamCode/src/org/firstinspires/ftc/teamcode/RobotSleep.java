package org.firstinspires.ftc.teamcode;

/**
 * 2019.10.26
 * Athena Z.
 */

public class RobotSleep implements RobotControl {
    long timeSleep;
    long timeStart;
    long timeNow;
    String memo = null;

    public RobotSleep(int timeSleep) {
        this.timeSleep = timeSleep;
    }

    public RobotSleep(int timeSleep, String memo) {
        this.memo = memo;
    }

    public String toString() {
        return (memo!=null?memo:"") + "Sleep for " + timeSleep;
    }

    @Override
    public void prepare() {
        timeStart = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        timeNow = System.currentTimeMillis();
    }

    @Override
    public void cleanUp() {

    }

    @Override
    public boolean isDone() {
        return timeNow > timeStart + timeSleep;
    }
}
