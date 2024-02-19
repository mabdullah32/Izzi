/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.odometry;

import com.stuypulse.stuylib.network.SmartBoolean;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.subsystems.vision.AprilTagVision;
import com.stuypulse.robot.util.vision.AprilTag;
import com.stuypulse.robot.util.vision.VisionData;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;

/** This class handles the odometry of the robot. */
public class Odometry extends SubsystemBase {

    private static final Odometry instance;

    static {
        instance = new Odometry();
    }

    public static Odometry getInstance() {
        return instance;
    }

    private final SwerveDriveOdometry odometry;
    private final SwerveDrivePoseEstimator estimator;
    private final SmartBoolean VISION_ACTIVE;

    private final Field2d field;
    private final FieldObject2d odometryPose2D;
    private final FieldObject2d estimatorPose2D;

    private Pose2d lastGoodPose;

    protected Odometry() {
        SwerveDrive swerve = SwerveDrive.getInstance();
        odometry = new SwerveDriveOdometry(
            swerve.getKinematics(),
            swerve.getGyroAngle(),
            swerve.getModulePositions(),
            new Pose2d());
        estimator = new SwerveDrivePoseEstimator(
            swerve.getKinematics(),
            swerve.getGyroAngle(),
            swerve.getModulePositions(),
            new Pose2d());

        VISION_ACTIVE = new SmartBoolean("Odometry/VISION ACTIVE", true);

        field = new Field2d();
        swerve.initFieldObject(field);
        odometryPose2D = field.getObject("Odometry Pose");
        estimatorPose2D = field.getObject("Estimator Pose");

        lastGoodPose = new Pose2d();

        SmartDashboard.putData("Field", field);
    }

    public Field2d getField() {
        return field;
    }

    public Pose2d getPose() {
        return estimator.getEstimatedPosition();
    }

    public double getDistanceToTag(AprilTag tag) {
        return getPose()
            .getTranslation()
            .getDistance(tag.getLocation().getTranslation().toTranslation2d());
    }

    /**
     * Reset the pose of the odometry to the given pose.
     *
     * @param pose the pose to reset to
     */
    public void reset(Pose2d pose) {
        SwerveDrive swerve = SwerveDrive.getInstance();
        odometry.resetPosition(swerve.getGyroAngle(), swerve.getModulePositions(), pose);
        estimator.resetPosition(swerve.getGyroAngle(), swerve.getModulePositions(), pose);
    }

    private void updateOdometry() {
        SwerveDrive swerve = SwerveDrive.getInstance();
        odometry.update(swerve.getGyroAngle(), swerve.getModulePositions());
        estimator.update(swerve.getGyroAngle(), swerve.getModulePositions());
    }

    private Matrix<N3, N1> getStdDevsFromArea(double areaPct) {
        areaPct = 1 - Math.pow(areaPct - 1, 2);

        SmartDashboard.putNumber("Vision/Transformed Pct", areaPct);

        final double MAX_AREA_PCT = 0.4;
        final double MIN_AREA_PCT = 0.05;

        final double MIN_TRANSLATION_STDDEVS = 0.1;
        final double MIN_ROTATION_STDDEVS = 0.1;

        final double MAX_TRANSLATION_STDDEVS = 0.5;
        final double MAX_ROTATION_STDDEVS = 0.5;

        // (MAX_AREA_PCT, MIN_TRANSLATION_STDDEVS)
        // (MIN_AREA_PCT, MAX_TRANSLATION_STDDEVS)

        final double TRANSLATION_SLOPE = (MIN_TRANSLATION_STDDEVS - MAX_TRANSLATION_STDDEVS) / (MAX_AREA_PCT - MIN_AREA_PCT);
        final double ROTATION_SLOPE = (MIN_ROTATION_STDDEVS - MAX_ROTATION_STDDEVS) / (MAX_AREA_PCT - MIN_AREA_PCT);

        double translationStdDev = TRANSLATION_SLOPE * (areaPct - MAX_AREA_PCT) + MIN_TRANSLATION_STDDEVS;
        double rotationStdDev = ROTATION_SLOPE * (areaPct - MAX_AREA_PCT) + MIN_TRANSLATION_STDDEVS;

        SmartDashboard.putNumber("Vision/Translation Std Dev", translationStdDev);
        SmartDashboard.putNumber("Vision/Rotation Std Dev", rotationStdDev);

        return VecBuilder.fill(translationStdDev, translationStdDev, rotationStdDev);
    }

    private void updateEstimatorWithVisionData(ArrayList<VisionData> outputs) {
        if (outputs.size() == 0) return;
        Pose2d avg = new Pose2d();
        double time = 0;
        for (VisionData data : outputs) {
            avg = new Pose2d(
                avg.getTranslation().plus(data.getPose().toPose2d().getTranslation()),
                avg.getRotation().plus(data.getPose().toPose2d().getRotation())
            );

            time += data.getTimestamp();

            // estimator.addVisionMeasurement(data.getPose().toPose2d(), data.getTimestamp());
        }
        estimator.addVisionMeasurement(avg.div(outputs.size()), time / (double)outputs.size());
    }

    @Override
    public void periodic() {
        ArrayList<VisionData> outputs = AprilTagVision.getInstance().getOutputs();

        updateOdometry();

        if (VISION_ACTIVE.get()) {
            updateEstimatorWithVisionData(outputs);
        }

        if (estimator.getEstimatedPosition().getTranslation().getNorm() > new Translation2d(Field.LENGTH, Field.WIDTH).getNorm() || 
            odometry.getPoseMeters().getTranslation().getNorm() > new Translation2d(Field.LENGTH, Field.WIDTH).getNorm() ||
            estimator.getEstimatedPosition().getX() == Double.NaN || estimator.getEstimatedPosition().getY() == Double.NaN ||
            odometry.getPoseMeters().getX() == Double.NaN || odometry.getPoseMeters().getY() == Double.NaN
        ) {
            reset(lastGoodPose);
        } else {
            lastGoodPose = getPose();
        }

        updateTelemetry();
    }

    private void updateTelemetry() {
        SmartDashboard.putNumber("Odometry/Odometry/X", odometry.getPoseMeters().getTranslation().getX());
        SmartDashboard.putNumber("Odometry/Odometry/Y", odometry.getPoseMeters().getTranslation().getY());
        SmartDashboard.putNumber("Odometry/Odometry/Rotation", odometry.getPoseMeters().getRotation().getDegrees());

        SmartDashboard.putNumber("Odometry/Estimator/X", estimator.getEstimatedPosition().getTranslation().getX());
        SmartDashboard.putNumber("Odometry/Estimator/Y", estimator.getEstimatedPosition().getTranslation().getY());
        SmartDashboard.putNumber("Odometry/Estimator/Rotation", estimator.getEstimatedPosition().getRotation().getDegrees());

        odometryPose2D.setPose(odometry.getPoseMeters());
        estimatorPose2D.setPose(estimator.getEstimatedPosition());
    }
}
