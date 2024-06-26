/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.conveyor;

import com.stuypulse.robot.commands.amper.AmperToHeight;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Amper.Lift;
import com.stuypulse.robot.subsystems.amper.Amper;
import com.stuypulse.robot.subsystems.conveyor.Conveyor;
import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.subsystems.shooter.Shooter;

import edu.wpi.first.wpilibj2.command.Command;

public class ConveyorToAmpUnsafe extends Command {
    public static Command withCheckLift() {
        return AmperToHeight.untilBelow(Lift.MIN_HEIGHT, Lift.MAX_HEIGHT_ERROR)
            .andThen(new ConveyorToAmp());
    }

    private final Conveyor conveyor;
    private final Shooter shooter;
    private final Intake intake;
    private final Amper amper;

    public ConveyorToAmpUnsafe() {
        conveyor = Conveyor.getInstance();
        shooter = Shooter.getInstance();
        intake = Intake.getInstance();
        amper = Amper.getInstance();

        addRequirements(conveyor, intake);
    }

    @Override
    public void initialize() {
        shooter.setTargetSpeeds(Settings.Shooter.HANDOFF);
    }

    @Override
    public void execute() {
        if (shooter.atTargetSpeeds()) {
            conveyor.toAmp();
            intake.acquire();
            amper.fromConveyor();
        }
    }

    @Override
    public boolean isFinished() {
        return amper.hasNote();
    }

    @Override
    public void end(boolean interrupted) {
        conveyor.stop();
        // shooter.stop();
        intake.stop();
        amper.stopRoller();
    }
}
