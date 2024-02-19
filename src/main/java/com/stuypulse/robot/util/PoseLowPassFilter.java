package com.stuypulse.robot.util;

import com.stuypulse.stuylib.streams.numbers.filters.LowPassFilter;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

public class PoseLowPassFilter {

    private Pose3d filteredPose;

    private LowPassFilter xFilter;
    private LowPassFilter yFilter;
    private LowPassFilter zFilter;
    private LowPassFilter rollFilter;
    private LowPassFilter pitchFilter;
    private LowPassFilter yawFilter;

    public PoseLowPassFilter(Number translationRC, Number rotationRC) {
        xFilter = new LowPassFilter(translationRC);
        yFilter = new LowPassFilter(translationRC);
        zFilter = new LowPassFilter(translationRC);
        rollFilter = new LowPassFilter(rotationRC);
        pitchFilter = new LowPassFilter(rotationRC);
        yawFilter = new LowPassFilter(rotationRC);

        filteredPose = new Pose3d();
    }

    public Pose3d update(Pose3d pose) {
        filteredPose = new Pose3d(
            new Translation3d(
                xFilter.get(pose.getX()),
                yFilter.get(pose.getY()),
                zFilter.get(pose.getZ())),
            new Rotation3d(
                rollFilter.get(pose.getRotation().getX()),
                pitchFilter.get(pose.getRotation().getY()),
                yawFilter.get(pose.getRotation().getZ())));
        
        return filteredPose;
    }

    public Pose3d get() {
        return filteredPose;
    }
    
}
