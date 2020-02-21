package org.firstinspires.ftc.suitbots;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class RangeSensor {
    private final ModernRoboticsI2cRangeSensor raw;
    private double last = 0.0;
    private boolean lastInf = false;

    public RangeSensor(final ModernRoboticsI2cRangeSensor _raw) {
        raw = _raw;
    }

    public double getReading() {
        double val = raw.getDistance(DistanceUnit.CM);
        if (val < 240.0) {
            last = val;
            lastInf = false;
        } else {
            lastInf = true;
        }
        return last;
    }

    public boolean isInfinite() {
        return lastInf;
    }

}
