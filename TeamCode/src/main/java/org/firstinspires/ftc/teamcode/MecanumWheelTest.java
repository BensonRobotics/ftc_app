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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="MecanumWheelTest", group="Linear Opmode")
//@Disabled
public class MecanumWheelTest extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    /*private DcMotor leftfront = null;
    private DcMotor rightfront = null;
    private DcMotor leftback = null;
    private DcMotor rightback = null;*/

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        double leftfrontPower;
        double rightfrontPower;
        double leftbackPower;
        double rightbackPower;

        /*leftfront  = hardwareMap.get(DcMotor.class, "left_drive");
        rightfront = hardwareMap.get(DcMotor.class, "right_drive");
        leftback  = hardwareMap.get(DcMotor.class, "left_drive");
        rightback = hardwareMap.get(DcMotor.class, "right_drive");

        leftfront.setDirection(DcMotor.Direction.FORWARD);
        rightfront.setDirection(DcMotor.Direction.FORWARD);
        leftfront.setDirection(DcMotor.Direction.FORWARD);
        rightfront.setDirection(DcMotor.Direction.FORWARD);*/

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double Rotate = gamepad1.right_stick_x;
            double Radius = Math.hypot(x,y);
            double stickTotal = Radius + Math.abs(Rotate);
            double Angle = Math.atan2(y,x) - Math.PI / 4;
            double cosAngle = Math.cos(Angle);
            double sinAngle = Math.sin(Angle);
            double LF_RB;  //leftfront and rightback motors
            double RF_LB;  //rightfront and leftback motors
            double multiplier;

            if (Math.abs(cosAngle) > Math.abs(sinAngle)) {   //scale the motor's speed so that at least one of them = 1
                multiplier = 1/Math.abs(cosAngle);//then add the rotate speed
                LF_RB = multiplier * cosAngle;
                RF_LB = multiplier * sinAngle;
            } else {
                multiplier = 1/Math.abs(sinAngle);
                LF_RB = multiplier * cosAngle;
                RF_LB = multiplier * sinAngle;
            }

            leftfrontPower    = LF_RB * Radius + Rotate;
            rightfrontPower   = RF_LB * Radius - Rotate;
            leftbackPower     = RF_LB * Radius + Rotate;
            rightbackPower    = LF_RB * Radius - Rotate;

            /*if (Rotate > 0) {                               //scale down with max = 1
                if (Math.abs(leftfrontPower) > Math.abs(leftbackPower)) {
                    multiplier = 1/Math.abs(leftfrontPower);
                } else {
                    multiplier = 1 /Math.abs(leftbackPower);
                }
            } else {
                if (Math.abs(rightbackPower) < Math.abs(rightfrontPower)) {
                    multiplier = 1/Math.abs(rightbackPower);
                } else {
                    multiplier = 1/Math.abs(rightfrontPower);
                }
            }

            //multiplier = 1;

            //multiply by the radius to get the correct speed
            leftfrontPower    = Range.clip(leftfrontPower * multiplier, -1.0, 1.0);
            rightfrontPower   = Range.clip(rightfrontPower * multiplier, -1.0, 1.0);
            leftbackPower     = Range.clip(leftbackPower * multiplier, -1.0, 1.0);
            rightbackPower    = Range.clip(rightbackPower * multiplier, -1.0, 1.0);*/
            if (Math.abs(stickTotal) > 1) {
                leftfrontPower    = leftfrontPower/stickTotal;
                rightfrontPower   = rightfrontPower/stickTotal;
                leftbackPower     = leftbackPower/stickTotal;
                rightbackPower    = rightbackPower/stickTotal;
            }

            /*leftfront.setPower(leftfrontPower);
            rightfront.setPower(rightfrontPower);
            leftback.setPower(leftbackPower);
            rightback.setPower(rightbackPower);*/

            // Show run time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("frontMotors", "left (%.2f), right (%.2f)", leftfrontPower, rightfrontPower);
            telemetry.addData("backMotors", "left (%.2f), right (%.2f)", leftbackPower, rightbackPower);
            telemetry.addData("Radius", Radius);
            telemetry.addData("stickTotal", stickTotal);
            telemetry.update();
        }
    }
}
