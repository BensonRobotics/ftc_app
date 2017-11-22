package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.text.SimpleDateFormat;
import java.util.Date;

/**
 * Created by FTC_Team_0267 on 10/23/2017.
 * By: Chase Hunt
 */
@Autonomous(name = "0267AUTONOMOUS",group = "Pushbot")
public class autonomous2017 extends LinearOpMode {
    private String startDate = new SimpleDateFormat("yyyy/MM/dd HH:mm:ss").format(new Date());;
    private ElapsedTime runtime = new ElapsedTime();
    Hardware267Bot robot = new Hardware267Bot();

    @Override
    public void runOpMode() throws InterruptedException {

        double  servoStartPosition = 0;

        robot.init(hardwareMap);

        robot.ballFlicker.setPosition(servoStartPosition);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        runtime.reset();

        sleep(14000);

        robot.leftMotor.setPower(0.4);
        robot.rightMotor.setPower(0.4);

        runtime.reset();

        while (opModeIsActive() && (runtime.seconds() < 2.0)) {
            telemetry.addData("Path", "Moving forward off of balancing stone.");
            telemetry.update();
        }

        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);

        telemetry.addData("Path", "Complete");

        telemetry.update();

        sleep(14000);
    }
}
