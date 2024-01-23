/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot;

import com.stuypulse.robot.commands.ConveyorCommand;
import com.stuypulse.robot.commands.auton.DoNothingAuton;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.subsystems.Conveyor.Conveyor;
import com.stuypulse.robot.subsystems.Conveyor.ConveyorLogic;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.input.gamepads.AutoGamepad;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {

    // Gamepads
    public final Gamepad driver = new AutoGamepad(Ports.Gamepad.DRIVER);
    public final Gamepad operator = new AutoGamepad(Ports.Gamepad.OPERATOR);
    
    // Subsystem
    public final Conveyor conveyor = Conveyor.getInstance();

    // Autons
    private static SendableChooser<Command> autonChooser = new SendableChooser<>();

    // Robot container

    public RobotContainer() {
        configureDefaultCommands();
        configureButtonBindings();
        configureAutons();
    }

    /****************/
    /*** DEFAULTS ***/
    /****************/

    private void configureDefaultCommands() {
        conveyor.setDefaultCommand(new ConveyorCommand(new ConveyorLogic.Default()));
    }

    /***************/
    /*** BUTTONS ***/
    /***************/

    private void configureButtonBindings() {
        operator.getDPadUp().onTrue(new ConveyorCommand(new ConveyorLogic.Default(), Conveyor.Direction.SHOOTER));
        operator.getDPadRight().onTrue(new ConveyorCommand(new ConveyorLogic.Default(), Conveyor.Direction.AMP));
        operator.getDPadDown().onTrue(new ConveyorCommand(new ConveyorLogic.Default(), Conveyor.Direction.NONE));
        operator.getRightTriggerButton().onTrue(new ConveyorCommand(new ConveyorLogic.ShooterShoot()));
    }

    /**************/
    /*** AUTONS ***/
    /**************/

    public void configureAutons() {
        autonChooser.setDefaultOption("Do Nothing", new DoNothingAuton());

        SmartDashboard.putData("Autonomous", autonChooser);
    }

    public Command getAutonomousCommand() {
        return autonChooser.getSelected();
    }
}
