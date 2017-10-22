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
    // color of the Robot Controller app to match the hue detected by the RGB sensor.
    int relativeLayoutId =
            hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
    final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

    // Set up strings for alliance selection

    String allianceTitle = "Alliance Selection";                      // The title of this selection
    String[] allianceOptions = new String[] {"Red", "Blue"};          // The options for this selection
    String allianceSelection;                                         // The alliance selection

    boolean b_button_pressed = false;                                 // "b" button not pressed
    boolean x_button_pressed = false;                                 // "x" button not pressed

    //----------------------------------------------------------------------------------------------
    // Main logic
    //----------------------------------------------------------------------------------------------

    @Override public void runOpMode() {

        // Alliance Selection - choose wisely

        telemetry.addLine(allianceTitle);
        telemetry.addLine("Press B for Red or X for Blue");
        telemetry.update();


        // loop until either the "b" button or the "x" button is pressed
        while (!b_button_pressed || !x_button_pressed) {

            // when the "b" button on the gamepad is pressed set alliance to RED
            if (gamepad1.b) {
                allianceSelection = allianceOptions[0];
                b_button_pressed = true;
            }
            // when the "x" button on the gamepad is pressed set the alliance to BLUE
            else if (gamepad1.x) {
                allianceSelection = allianceOptions[1];
                x_button_pressed = true;
            }
        }

        telemetry.addLine().addData("Alliance Selection: ",allianceSelection);
        telemetry.update();

        // change the background color to match the color detected by the RGB sensor.
        // pass a reference to the hue, saturation, and value array as an argument
        // to the HSVToColor method.
        relativeLayout.post(new Runnable() {
            public void run() {
                if (allianceSelection == allianceOptions[0]) {
                    relativeLayout.setBackgroundColor(Color.RED);
                }
                else {
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
    }
}
