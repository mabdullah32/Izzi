package com.stuypulse.robot.util.filterpractice;

import java.util.ArrayList;
import java.util.Collections;

import com.stuypulse.stuylib.streams.numbers.IStream;
import com.stuypulse.stuylib.streams.numbers.filters.IFilter;

public class MedianFilter implements IFilter {
    
    private final int windowSize;
    // private final double[] cBuffer;
    // private int index = 0;
    private ArrayList<Double> values;

    public MedianFilter(int windowSize){
        this.windowSize = windowSize;
        values = new ArrayList<Double>();
    }

    @Override
    public double get(double x) {
        values.add(x);
        if (values.size() > windowSize) {
            values.remove(0);
        }

        ArrayList<Double> temp = new ArrayList<Double>(values);
        Collections.sort(temp);

        int size = temp.size();
        if (size % 2 == 0) {
            return (temp.get(size / 2 - 1) + temp.get(size / 2)) / 2;
        }
        else {
            return temp.get(size / 2);
        }
    }

    public static void main(String... args) {
        IStream camera_raw = new Camera();
        IStream camera_filtered = camera_raw.filtered(new MedianFilter(5));

        System.out.print("Raw Camera Values: ");
        for (int i = 0; i < 30; ++i) System.out.printf("%.2f, ", camera_raw.get());
        System.out.println("\n");

        System.out.print("Filtered Camera Values: ");
        for (int i = 0; i < 30; ++i) System.out.printf("%.2f, ", camera_filtered.get());
        System.out.println("\n");
    }
    
}
