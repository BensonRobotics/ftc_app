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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.text.SimpleDateFormat;
import java.util.Date;
/**
 * Created by FTC_Team_0267 on 10/30/2018.
 * By: Jillian Hogan
 */
@Autonomous(name = "lazy forwards",group = "Pushbot")
public class depot_autonomous extends LinearOpMode {
    private String startDate = new SimpleDateFormat("yyyy/MM/dd HH:mm:ss").format(new Date());;
    private ElapsedTime runtime = new ElapsedTime();
    Hardware267Bot robot = new Hardware267Bot();
    @Override
    public void runOpMode() throws InterruptedException {
        runtime.reset();
        robot.init(hardwareMap);
        while (!opModeIsActive()) {
            sleep(1000);
        }
        /*
        robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.armMotor.setTargetPosition(80);
        robot.armMotor.setPower(1);
        while (robot.armMotor.getCurrentPosition() < 80) {}
        robot.armMotor.setPower(0);
        robot.armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        */
            waitForStart();
            robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.liftMotor.setTargetPosition(6231);

            robot.liftMotor.setPower(0.6);

            sleep(10000);
            //going away from lander
            robot.leftMotor.setPower(-0.5);
            robot.rightMotor.setPower(-0.5);

            sleep(2000);


        /*
        robot.leftMotor.setPower(0.5);
        robot.rightMotor.setPower(-0.5);
        sleep(700);
        robot.leftMotor.setPower(-0.5);
        robot.rightMotor.setPower(-0.5);
        sleep(3000);
        robot.leftMotor.setPower(-0);
        robot.rightMotor.setPower(-0);
        */
    }
}
