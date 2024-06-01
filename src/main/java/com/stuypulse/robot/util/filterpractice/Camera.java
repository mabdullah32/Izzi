package com.stuypulse.robot.util.filterpractice;

import java.util.Random;

import com.stuypulse.stuylib.streams.numbers.IStream;

public class Camera implements IStream {

    private Random rand;

    public Camera() {
        rand = new Random();
    }

    public double get() {
        double x = 4.0 + 2.0 * Math.sin(System.currentTimeMillis() / 2000.0);

        if (rand.nextInt(10) == 0) {
            x *= 0.25;
        } else if(rand.nextInt(10) == 0) {
            x *= 4.0;
        }

        return x;
    }

}