/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

/**
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a POV Game style Teleop for a PushBot
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="267bot: Teleop ", group="267")
//@Disabled
public class team267botTeleop_Linear extends LinearOpMode {

    /* Declare OpMode members. */
    Hardware267Bot robot           = new Hardware267Bot();   // Use a Pushbot's hardware
    public static final double TRIGGER_MIN = 0; //TODO: Find value
    public static final double TRIGGER_MAX = 1; //TODO: Find value                                                 // could also use HardwarePushbotMatrix class.

    @Override
    public void runOpMode() {
        double left;
        double right;
        double leftBeltPower;
        double rightBeltPower;
        double leftFeederPower;
        double rightFeederPower;



        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "AHGGHGGGHHGH");    //
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
            left = -gamepad1.left_stick_y;
            right = -gamepad1.right_stick_y;
           // left = Range.clip(left, -.5, .5);
          //  right= Range.clip(right, -.5, .5);

            // Output the safe vales to the motor drives.
            robot.leftMotor.setPower(left);
            robot.rightMotor.setPower(right);

            if (gamepad1.left_trigger >0) {
                //If the left trigger is pressed, move block forward.

                //normalize to between 0 and 1

                //Belts
                leftBeltPower=  gamepad1.left_trigger;
                rightBeltPower= gamepad1.left_trigger;

                //Feeders

                leftFeederPower= gamepad1.left_trigger;
                rightFeederPower= gamepad1.right_trigger;

            }
            else if (gamepad1.right_trigger >0) {
                //If the right trigger is pressed, move block backwards.

                //belts
                leftBeltPower= -gamepad1.right_trigger;
                rightBeltPower= -gamepad1.right_trigger;

                //feeder
                leftFeederPower= -gamepad1.left_trigger;
                rightFeederPower= -gamepad1.right_trigger;
            }
            else {
                //If neither triggers are pressed, do nothing.

                //belts
                leftBeltPower= 0;
                rightBeltPower= 0;

                //feeder
                leftFeederPower= 0;
                rightFeederPower= 0;
            }
            robot.leftBelt.setPower(leftBeltPower);
            robot.rightBelt.setPower(rightBeltPower);


            // Send telemetry message to signify robot running;

            telemetry.addData("left",  "%.2f", left);
            telemetry.addData("right", "%.2f", right);
            telemetry.addData("leftBelt", "%.2f", leftBeltPower);
            telemetry.addData("rightBelt","%.2f", rightBeltPower);
            telemetry.addData("leftTrigger", "%.2f", gamepad1.left_trigger);
            telemetry.addData("rightTrigger","%.2f", gamepad1.right_trigger);
            telemetry.update();

            // Pace this loop so jaw action is reasonable speed.
            sleep(50);
        }
    }
}
