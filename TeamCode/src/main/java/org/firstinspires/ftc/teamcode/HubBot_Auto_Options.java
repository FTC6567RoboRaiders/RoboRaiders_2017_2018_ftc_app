/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


/**
 * {@link HubBot_Auto_Options} prototype for autonomous selection of options, things
 * like alliance, how many glyphs to obtain and place, location, etc....
 *
 * more later....
 *
 *
 **/

@Autonomous(name = "HubBot: Autonomous Options", group = "Auto")

public class HubBot_Auto_Options extends LinearOpMode
{

    // The following is used to change the background color of the robot controller
    // get a reference to the RelativeLayout so we can change the background
    // color of the Robot Controller app to match the alliance selection.
    View relativeLayout;

    // Set up strings for alliance selection

    String allianceTitle = "Alliance Selection";                      // The title of this selection
    String[] allianceOptions = new String[] {"Red", "Blue"};          // The options for this selection
    String allianceSelection;                                         // The alliance selection

    boolean cur_B_ButtonState;                                        // "b" button current state
    boolean cur_X_ButtonState;                                        // "x" button current state

    boolean prev_B_ButtonState;                                       // "b" button previous state
    boolean prev_X_ButtonState;                                       // "x" button previous state

    //----------------------------------------------------------------------------------------------
    // Main logic
    //----------------------------------------------------------------------------------------------

    @Override public void runOpMode() {

        // Get a reference to the RelativeLayout so we can later change the background
        // color of the Robot Controller app to match the alliance selection
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);


        // Alliance Selection - choose wisely

        telemetry.addLine(allianceTitle);
        telemetry.addLine("Press B for Red or X for Blue");
        telemetry.update();

        gamepad1.reset();                                             // reset the gamepad to initial state

        /*
        Default the previous button states for the "B" and "X" button to indicate that the
        buttons were not pushed.
         */
        prev_X_ButtonState = false;
        prev_B_ButtonState = false;


        // loop until either the "b" button or the "x" button is pressed
        // the logic here says OR the previous button states and when they are both false continue
        // here is a table of how this works
        /*
             +--------------------+------+--------------------+--------+-------------+
             | prev_B_ButtonState | -OR- | prev_X_ButtonState | Result | Neg. Result |
             +--------------------+------+--------------------+--------+-------------+
             |      FALSE         | -OR- |     FALSE          | FALSE  |   TRUE      |
             +--------------------+------+--------------------+--------+-------------+
             |      FALSE         | -OR- |     TRUE           | TRUE   |   FALSE     |
             +--------------------+------+--------------------+--------+-------------+
             |      TRUE          | -OR- |     FALSE          | TRUE   |   FALSE     |
             +--------------------+------+--------------------+--------+-------------+
             |      TRUE          | -OR- |     TRUE           | TRUE   |   FALSE     |
             +--------------------+------+--------------------+--------+-------------+

         */
        while ( !(prev_B_ButtonState | prev_X_ButtonState) ) {

            cur_B_ButtonState = gamepad1.b;                           // get the current state of button b
            cur_X_ButtonState = gamepad1.x;                           // get the current state of button x

            if (cur_B_ButtonState) {                                  // when the "b" button on the gamepad is pressed set alliance to RED
                if (!prev_B_ButtonState) {                            // when the previous "b" button was NOT pushed
                    allianceSelection = allianceOptions[0];           // set alliance selection to RED
                    prev_B_ButtonState = true;                        // indicate that the previous B button state is PUSHED
                }
            }

            else if (cur_X_ButtonState) {                             // when the "X" button on the gamepad is pressed set the alliance to BLUE
                if (!prev_X_ButtonState) {                            // when the previous "x" button was NOT pushed
                    allianceSelection = allianceOptions[1];           // set alliance selection to BLUE
                    prev_X_ButtonState = true;                        // indicate that the previous X button state is PUSHED
                }
            }

            telemetry.update();                                       // so when this line is removed we get a problem with
                                                                      // the state of the prev variables...not sure what java/android
                                                                      // thinks is going on here...more investigation is needed
        }

        telemetry.addLine().addData("Alliance Selection: ",allianceSelection);
        telemetry.update();


        // change the background color to match the alliance selection
        relativeLayout.post(new Runnable() {
            public void run() {
                if ( allianceSelection.equals(allianceOptions[0]) ) { // alliance selection is RED
                    relativeLayout.setBackgroundColor(Color.RED);
                }
                else {                                                // alliance selection is BLUE
                    relativeLayout.setBackgroundColor(Color.BLUE);
                }
            }
        });

        // Wait until we're told to go
        waitForStart();

        // Loop and update the dashboard
        while (opModeIsActive()) {
            telemetry.update();
        }

        // change the background color back to white
        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.WHITE);

            }
        });

    }
}
