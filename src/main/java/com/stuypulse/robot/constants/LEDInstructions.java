/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.constants;

import com.stuypulse.robot.subsystems.leds.instructions.LED694;
import com.stuypulse.robot.subsystems.leds.instructions.LEDInstruction;
import com.stuypulse.robot.subsystems.leds.instructions.LEDPulseColor;
import com.stuypulse.robot.subsystems.leds.instructions.LEDRainbow;
import com.stuypulse.robot.subsystems.leds.instructions.LEDSection;
import com.stuypulse.robot.subsystems.leds.instructions.LEDSingleColor;
import com.stuypulse.robot.subsystems.leds.instructions.RichieMode;
import com.stuypulse.robot.util.SLColor;

public interface LEDInstructions {

    /**************/
    /*** STATIC ***/
    /**************/

    LEDInstruction OFF = new LEDSingleColor(new SLColor(0, 0, 0));

    LEDInstruction AQUA = new LEDSingleColor(new SLColor(0, 255, 255));
    LEDInstruction BLACK = new LEDSingleColor(new SLColor(0, 0, 0));
    LEDInstruction BLUE = new LEDSingleColor(new SLColor(0, 128, 255));
    LEDInstruction BLUE_GREEN = new LEDSingleColor(new SLColor(0, 255, 128));
    LEDInstruction BLUE_VIOLET = new LEDSingleColor(new SLColor(51, 51, 255));
    LEDInstruction DARK_BLUE = new LEDSingleColor(new SLColor(0, 0, 204));
    LEDInstruction DARK_GRAY = new LEDSingleColor(new SLColor(64, 64, 64));
    LEDInstruction DARK_GREEN = new LEDSingleColor(new SLColor(0, 153, 0));
    LEDInstruction DARK_RED = new LEDSingleColor(new SLColor(204, 0, 0));
    LEDInstruction GOLD = new LEDSingleColor(new SLColor(218, 165, 32));
    LEDInstruction GRAY = new LEDSingleColor(new SLColor(128, 128, 128));
    LEDInstruction GREEN = new LEDSingleColor(new SLColor(0, 255, 0));
    LEDInstruction HOT_PINK = new LEDSingleColor(new SLColor(255, 105, 180));
    LEDInstruction LAWN_GREEN = new LEDSingleColor(new SLColor(102, 204, 0));
    LEDInstruction LIME = new LEDSingleColor(new SLColor(191, 255, 0));
    LEDInstruction ORANGE = new LEDSingleColor(new SLColor(255, 128, 0));
    LEDInstruction PINK = new LEDSingleColor(new SLColor(255, 192, 203));
    LEDInstruction PURPLE = new LEDSingleColor(new SLColor(160, 32, 240));
    LEDInstruction RED = new LEDSingleColor(new SLColor(255, 0, 0));
    LEDInstruction RED_ORANGE = new LEDSingleColor(new SLColor(255, 83, 73));
    LEDInstruction VIOLET = new LEDSingleColor(new SLColor(127, 0, 255));
    LEDInstruction WHITE = new LEDSingleColor(new SLColor(255, 255, 255));
    LEDInstruction YELLOW = new LEDSingleColor(new SLColor(255, 255, 0));

    /******************/
    /*** NON-STATIC ***/
    /******************/

    LEDInstruction RAINBOW = new LEDRainbow();

    LEDInstruction PULSE_RED = new LEDPulseColor(SLColor.RED);
    LEDInstruction PULSE_RED_BLUE = new LEDPulseColor(SLColor.RED, SLColor.BLUE);
    LEDInstruction RICHIE = new RichieMode(SLColor.RED);

    /********************************************/
    /*** LED CONSTANTS TO BE USED IN COMMANDS ***/
    /********************************************/

    LEDInstruction DEFAULT = LEDInstructions.OFF;

    LEDInstruction INTAKE = new LEDPulseColor(SLColor.BLUE);

    LEDInstruction PICKUP = new LEDPulseColor(SLColor.RED);

    LEDInstruction SPEAKER_ALIGN = new LEDPulseColor(SLColor.GREEN);

    LEDInstruction AMP_ALIGN = new LEDPulseColor(SLColor.GREEN);
    
    LEDInstruction AMP_SCORE = LEDInstructions.PURPLE;

    LEDInstruction AUTO_SWERVE =  new LEDPulseColor(SLColor.GREEN);

    LEDInstruction ATTENTION = new LED694(0.01, SLColor.BLUE);

    LEDInstruction CLIMB_UP = new LEDSection(false, SLColor.PURPLE, SLColor.GREEN);

    LEDInstruction TRAP_ALIGN = new LEDSection(true, SLColor.PURPLE, SLColor.GREEN);

    LEDInstruction FERRY = new LEDPulseColor(SLColor.ORANGE); // No Ferry command yet

	LEDInstruction CONTAINS_NOTE = new LEDSingleColor(SLColor.RED);

    // TO FUTURE USERS, DONT PUT LEDAlign and LEDAutonChooser (any disabled LEDInstructions) inside
    // the LEDInstructions interface
}
