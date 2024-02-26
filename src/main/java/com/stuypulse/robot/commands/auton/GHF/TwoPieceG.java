package com.stuypulse.robot.commands.auton.GHF;

import com.stuypulse.robot.commands.auton.FollowPathAlignAndShoot;
import com.stuypulse.robot.commands.auton.FollowPathAndIntake;
import com.stuypulse.robot.commands.conveyor.ConveyorShootRoutine;
import com.stuypulse.robot.commands.shooter.ShooterPodiumShot;
import com.stuypulse.robot.commands.swerve.SwerveDriveToPose;
import com.stuypulse.robot.constants.Settings.Auton;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class TwoPieceG extends SequentialCommandGroup {

    public TwoPieceG() {
        addCommands(
            new ParallelCommandGroup(
                new WaitCommand(Auton.SHOOTER_STARTUP_DELAY)
                    .andThen(new ShooterPodiumShot()),

                SwerveDriveToPose.speakerRelative(-50)
                    .withTolerance(0.1, 0.1, 2)
            ),

            new ConveyorShootRoutine(),

            new FollowPathAndIntake("Start To G (GHF)"),
            new FollowPathAlignAndShoot("G To GShoot (GHF)", SwerveDriveToPose.speakerRelative(-45))
        );
    }

}
