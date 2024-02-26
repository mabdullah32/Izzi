/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot;

import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.input.gamepads.AutoGamepad;

import com.stuypulse.robot.commands.*;
import com.stuypulse.robot.commands.amper.*;
import com.stuypulse.robot.commands.auton.*;
import com.stuypulse.robot.commands.auton.CBADE.*;
import com.stuypulse.robot.commands.auton.GHF.*;
import com.stuypulse.robot.commands.auton.HGF.*;
import com.stuypulse.robot.commands.auton.tests.*;
import com.stuypulse.robot.commands.climber.*;
import com.stuypulse.robot.commands.conveyor.*;
import com.stuypulse.robot.commands.intake.*;
import com.stuypulse.robot.commands.leds.*;
import com.stuypulse.robot.commands.notealignment.*;
import com.stuypulse.robot.commands.shooter.*;
import com.stuypulse.robot.commands.swerve.*;
import com.stuypulse.robot.constants.LEDInstructions;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Driver;
import com.stuypulse.robot.constants.Settings.Swerve.Assist;
import com.stuypulse.robot.subsystems.amper.Amper;
import com.stuypulse.robot.subsystems.climber.*;
import com.stuypulse.robot.subsystems.conveyor.Conveyor;
import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.subsystems.leds.LEDController;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.subsystems.vision.AprilTagVision;
import com.stuypulse.robot.subsystems.vision.NoteVision;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {

    // Gamepads
    public final Gamepad driver = new AutoGamepad(Ports.Gamepad.DRIVER);
    public final Gamepad operator = new AutoGamepad(Ports.Gamepad.OPERATOR);

    public final AprilTagVision vision = AprilTagVision.getInstance();
    public final NoteVision noteVision = NoteVision.getInstance();
    public final Odometry odometry = Odometry.getInstance();

    public final Climber climber = Climber.getInstance();
    public final Amper amper = Amper.getInstance();
    public final LEDController leds = LEDController.getInstance();
    public final Conveyor conveyor = Conveyor.getInstance();
    public final Intake intake = Intake.getInstance();
    public final Shooter shooter = Shooter.getInstance();
    public final SwerveDrive swerve = SwerveDrive.getInstance();

    // Autons
    private static SendableChooser<Command> autonChooser;

    // RobotContainer
    public RobotContainer() {
        autonChooser = new SendableChooser<Command>();

        configureDefaultCommands();
        configureButtonBindings();
        configureAutons();

        SmartDashboard.putData("Gamepads/Driver", driver);
        SmartDashboard.putData("Gamepads/Operator", operator);
    }

    /****************/
    /*** DEFAULTS ***/
    /****************/

    private void configureDefaultCommands() {
        swerve.setDefaultCommand(new SwerveDriveDrive(driver));
        leds.setDefaultCommand(new LEDDefaultMode());
    }

    /***************/
    /*** BUTTONS ***/
    /***************/

    private void configureButtonBindings() {
        configureOperatorBindings();
        configureDriverBindings();
    }

    private void configureDriverBindings() {
        // intaking with swerve pointing at note
        driver.getRightTriggerButton()
            .whileTrue(new IntakeAcquire().andThen(new BuzzController(driver)))
            // .whileTrue(new SwerveDriveNoteAlignedDrive(driver))
            .whileTrue(new LEDSet(LEDInstructions.DARK_BLUE));

        // note to shooter and align
        // then shoot
        driver.getRightBumper()
            .onTrue(new ShooterPodiumShot())
            .whileTrue(new WaitCommand(Settings.Shooter.TELEOP_SHOOTER_STARTUP_DELAY)
                .andThen(new SwerveDriveToShoot()
                    .deadlineWith(new LEDSet(LEDInstructions.ASSIST_FLASH)))
                .andThen(new ShooterWaitForTarget())
                .andThen(new ConveyorShoot()))
            .onFalse(new ConveyorStop())
            .onFalse(new IntakeStop())
            .onFalse(new ShooterStop());

        // note to amper and align then score
        driver.getLeftBumper()
            .whileTrue(new SwerveDriveAmpAlign())
            .onFalse(new AmperToHeight(Settings.Amper.Lift.MIN_HEIGHT))
            .onFalse(new AmperStop());

        // score speaker no align
        driver.getRightMenuButton()
            .onTrue(new ShooterPodiumShot())
            .onTrue(new ShooterWaitForTarget().andThen(new ConveyorShoot()))
            .onFalse(new ConveyorStop())
            .onFalse(new IntakeStop())
            .onFalse(new ShooterStop());
            
        // score amp no align
        driver.getLeftMenuButton()
            .whileTrue(new ConveyorToAmp()
                .andThen(new AmperScore()))
            .onFalse(new AmperStop());

        driver.getDPadUp()
            .onTrue(new ClimberToTop());
        // driver.getDPadDown()
        //     .onTrue(new ClimberToBottom());

        driver.getDPadRight()
            .onTrue(new SwerveDriveResetHeading());

        // driver.getRightButton()
        //     .whileTrue(new ClimberSetupRoutine());
        // driver.getBottomButton()
        //     .whileTrue(new ClimberScoreRoutine());

        driver.getTopButton()
            // on command start
            .onTrue(new BuzzController(driver, Assist.BUZZ_INTENSITY)
                .deadlineWith(new LEDSet(LEDInstructions.GREEN)))
                
            .onTrue(new SwerveDriveAutomatic(driver)
                // after command end
                .andThen(new BuzzController(driver, Assist.BUZZ_INTENSITY)
                    .deadlineWith(new LEDSet(LEDInstructions.GREEN)))

                .andThen(new WaitCommand(Driver.Drive.BUZZ_DURATION))
                
                .andThen(new BuzzController(driver, Assist.BUZZ_INTENSITY)
                    .deadlineWith(new LEDSet(LEDInstructions.GREEN))));
    }

    private void configureOperatorBindings() {
        new Trigger(() -> operator.getLeftStick().magnitude() > Settings.Operator.DEADBAND.get())
                .whileTrue(new ClimberDrive(operator));

        new Trigger(() -> operator.getRightStick().magnitude() > Settings.Operator.DEADBAND.get())
                .whileTrue(new AmperLiftDrive(operator));

        operator.getLeftTriggerButton()
            .onTrue(new IntakeDeacquire())
            .onFalse(new IntakeStop());
        operator.getRightTriggerButton()
            .whileTrue(new IntakeAcquire().andThen(new BuzzController(driver)))
            .whileTrue(new LEDSet(LEDInstructions.DARK_BLUE));

        operator.getLeftBumper()
            .onTrue(ConveyorToAmp.withCheckLift())
            .onFalse(new ConveyorStop())
            .onFalse(new IntakeStop())
            .onFalse(new AmperStop());

        operator.getTopButton()
            .onTrue(new AmperScore())
            .onTrue(new ConveyorShoot())
            .onFalse(new AmperStop())
            .onFalse(new ConveyorStop())
            .onFalse(new IntakeStop());

        operator.getDPadUp()
            .whileTrue(new AmperLiftFineAdjust(operator));
        operator.getDPadDown()
            .whileTrue(new AmperLiftFineAdjust(operator));

        operator.getDPadRight()
            .onTrue(new ClimberToTop());
        // operator.getDPadLeft()
        //     .onTrue(new ClimberToBottom());

        operator.getLeftButton()
                .onTrue(new AmperToHeight(Settings.Amper.Lift.TRAP_SCORE_HEIGHT));
        operator.getRightButton()
                .onTrue(new AmperToHeight(Settings.Amper.Lift.AMP_SCORE_HEIGHT));
        operator.getBottomButton()
            .onTrue(new AmperToHeight(Settings.Amper.Lift.MIN_HEIGHT));

        // human player attention button
        operator.getRightMenuButton()
            .whileTrue(new LEDSet(LEDInstructions.PULSE_PURPLE));
        
        operator.getLeftMenuButton()
            .onTrue(new AmperIntake())
            .onFalse(new AmperStop());
    }

    /**************/
    /*** AUTONS ***/
    /**************/

    public void configureAutons() {
        autonChooser.addOption("Do Nothing", new DoNothingAuton());

        autonChooser.addOption("Mobility", new Mobility());

        autonChooser.addOption("5 Piece CBAE", new FivePieceCBAE());
        autonChooser.setDefaultOption("4 Piece CBA", new FourPieceCBA());
        autonChooser.addOption("3 Piece CB", new ThreePieceCB());
        autonChooser.addOption("2 Piece C", new TwoPieceC());

        autonChooser.addOption("4 Piece HGF", new FourPieceHGF());
        autonChooser.addOption("3 Piece HG", new ThreePieceHG());
        autonChooser.addOption("2 Piece H", new TwoPieceH());

        SmartDashboard.putData("Autonomous", autonChooser);
    }

    public Command getAutonomousCommand() {
        return autonChooser.getSelected();
    }

    public static String getAutonomousCommandNameStatic() {
        if (autonChooser.getSelected() == null) {
            return "Do Nothing";
        }
        
        return autonChooser.getSelected().getName();
    }
}
