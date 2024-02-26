package com.stuypulse.robot.commands.auton.CBADE;

import com.stuypulse.robot.commands.auton.FollowPathAlignAndShoot;
import com.stuypulse.robot.commands.auton.FollowPathAndIntake;
import com.stuypulse.robot.commands.conveyor.ConveyorShootRoutine;
import com.stuypulse.robot.commands.intake.IntakeAcquire;
import com.stuypulse.robot.commands.shooter.ShooterPodiumShot;
import com.stuypulse.robot.commands.shooter.ShooterStop;
import com.stuypulse.robot.commands.swerve.SwerveDriveToPose;
import com.stuypulse.robot.commands.swerve.SwerveDriveToShoot;
import com.stuypulse.robot.constants.Settings.Auton;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class FourPieceCBA extends SequentialCommandGroup {

    public FourPieceCBA() {
        addCommands(
            new ParallelCommandGroup(
                new WaitCommand(Auton.SHOOTER_STARTUP_DELAY)
                    .andThen(new ShooterPodiumShot()),
                
                new SwerveDriveToShoot(-37)
                    .withTimeout(2.5)
            ),

            new ConveyorShootRoutine(),

            new IntakeAcquire().withTimeout(Auton.DEFAULT_INTAKE_TIMEOUT),
            new SwerveDriveToPose(() -> SwerveDriveToShoot.getSpeakerTargetPose(2.9))
                .withTolerance(0.1, 0.1, 5),
            new ConveyorShootRoutine(),

            new FollowPathAndIntake("C to B"),
            new SwerveDriveToShoot(5),
            new ConveyorShootRoutine(),

            // TODO: reduce angle tolerance on last shot
            new FollowPathAndIntake("B To A"),
            new SwerveDriveToShoot(35),
            new ConveyorShootRoutine(),

            new ShooterStop()
        );
    }
    
}
