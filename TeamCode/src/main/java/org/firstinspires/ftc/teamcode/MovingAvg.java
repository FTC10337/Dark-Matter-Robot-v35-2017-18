package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.internal.collections.ArrayRunQueue;

/**
 * Created by jholly on 12/12/2017.
 */

public class MovingAvg {

    int maxSamp = 0;
    double sum = 0.0;
    ArrayRunQueue<Double> samples;

    public MovingAvg (int max) {
        maxSamp = max;
        samples = new ArrayRunQueue<Double>();
    }

    public void add(Double val) {
        if (!Double.isNaN(val)) {
            samples.addLast(val);
            sum += val;
        } else {
            // Got a NaN -- remove 1 old sample so we continue to process queue
            sum -= removeFirst();
        }
        if (samples.size() > maxSamp) {
            sum -= removeFirst();
        }
    }

    public Double removeFirst() {
        if (samples.size() > 0) {
            return samples.removeFirstCount(1);
        } else {
            return 0.0;
        }
    }

    public double average() {
        if (samples.size() > 0) {
            return (sum / samples.size());
        } else {
            return 9999.9;
        }
    }

}
