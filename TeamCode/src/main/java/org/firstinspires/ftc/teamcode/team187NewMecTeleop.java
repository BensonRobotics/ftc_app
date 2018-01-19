package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="MecBot:servo test ", group="MecBot")

public class team187NewMecTeleop extends OpMode{

    /* Declare OpMode members. */
    Hardware187NewBot robot       = new Hardware187NewBot(); // use the class created to define a Pushbot's hardware
                                                         // could also use HardwarePushbotMatrix class.

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */

    public double griprightPos  = robot.RIGHT_SERVO_OPEN ;
    public double gripleftPos  = robot.LEFT_SERVO_OPEN ;

    @Override

    public void start() {

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
//    double legacyAngle;
    @Override
    public void loop() {
        double joyX;
        double joyY;
        double rightX;
        double r;
        double robotAngle;

        double fl,fr,bl,br;



        final double ARM_UP_POWER    =  0.45 ;
        final double ARM_DOWN_POWER  = -0.45 ;





        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)

        joyY = gamepad1.left_stick_y;
        joyX = -gamepad1.left_stick_x;
        rightX = -gamepad1.right_stick_x;

        r = Math.hypot(joyX, joyY);

        robotAngle = Math.atan2(joyY,joyX) - Math.PI / 4;

//        legacyAngle += rightX/188.318;
//
//        if(gamepad1.right_bumper) {
//            legacyAngle += Math.toRadians(5);
//        }
//        if(gamepad1.left_bumper) {
//            legacyAngle -= Math.toRadians(5);
//        }

        fl = r * Math.cos(robotAngle) + rightX;
        fr = r * Math.sin(robotAngle) - rightX;
        bl = r * Math.sin(robotAngle) + rightX;
        br = r * Math.cos(robotAngle) - rightX;

        robot.frontLeft.setPower(fl);
        robot.frontRight.setPower(fr);
        robot.backLeft.setPower(bl);
        robot.backRight.setPower(br);

        if (gamepad1.x) {
            robot.gripright.setPosition(robot.RIGHT_SERVO_OPEN);
            robot.gripleft.setPosition(robot.LEFT_SERVO_OPEN);
        }

        else if (gamepad1.b)
        {
            robot.gripright.setPosition(robot.RIGHT_SERVO_CLOSED);
            robot.gripleft.setPosition(robot.LEFT_SERVO_CLOSED);
        }


        if (gamepad1.y)
            robot.liftLow.setPower(ARM_UP_POWER);
        else if (gamepad1.a)
            robot.liftLow.setPower(ARM_DOWN_POWER);
        else
            robot.liftLow.setPower(0.0);

        // Send telemetry message to signify robot running;
        telemetry.addData("Robot Angle",  "%.2f", Math.toDegrees(robotAngle));
        telemetry.addData("Robot Angle",  "%.2f", rightX);
//        telemetry.addData("Legacy Angle",  "%.2f", Math.toDegrees(legacyAngle));
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
