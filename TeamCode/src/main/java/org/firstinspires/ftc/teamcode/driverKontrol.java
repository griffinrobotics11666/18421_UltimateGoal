package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="18421 Blaster TWO", group="UltimateGoalBot")
//@Disabled
public class driverKontrol extends OpMode{

    /* Declare OpMode members. */
    Hardwarebot2 robot       = new Hardwarebot2(); // use the class created to define a Pushbot's hardware
    // rt = 75% power for shooter
    // rb = 100% power for the shooter
    // lb = feeder goes backward
    // lt = feeder goes forward
    // b = for the servo to "go go power rangers" at 90 degrees

    //Constant variables
    final double feederMotorMinPosition = -1.0;
    final double feederMotorSetPosition = 0.0;
    final double feederMotorMaxPosition = 1.0;
    double loaderServoSetPosition = 0.0;
    final double loaderServoMaxPosition = 0.4;
    final double shooterMotorMinPower = 0.0;
    // i hate fianl
    final double shooterMotorMidPower = 0.75;
    final double shooterMotorMaxPower = 1.0; // eazy
    final double startContinuous = 0.0;
    final double endContinuous = 1.0;
    final double rotationStart10 = 0;
    final  double rotationEnd10 = 0.5;
    final double rotationStart25 = 0;
    final  double rotationEnd25 = 0.25;
    final boolean debug = true;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        telemetry.addData("Status" , "Initialized");
        robot.init(hardwareMap); //do we need this??????

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
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        double y = -gamepad1.left_stick_y; // remember this iz reversed
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;
        robot.leftFront.setPower(y + x + rx);
        robot.rightFront.setPower(y - x - rx);
        robot.leftBack.setPower( y - x + rx);
        robot.rightBack.setPower( y + x - rx);

        // we use rt, rb, lt, lb to run shooter  and feeder motor bois
        if (gamepad1.right_trigger > 0.5) {
            robot.shooter.setPower(0.95);
        } else if ( gamepad1.right_bumper) {
            robot.shooter.setPower(1.0);
        }  else  {
            robot.shooter.setPower(0.0);
        }


        if (gamepad1.left_trigger > 0.5) {
            robot.feeder.setPower(1.0);
        } else if (gamepad1.left_bumper){
            robot.feeder.setPower(-1.0);
        }  else {
            robot.feeder.setPower(0.0);
        }

        // we use b 4 da servo
        if (gamepad1.b) {
            robot.loaderServo.setPosition(loaderServoSetPosition); // 0.4

        } else {                                                    // we did this so it goes with our OG plan
            robot.loaderServo.setPosition(loaderServoMaxPosition);  // 0.0
        }
        if (gamepad1.a){
            robot.servoRotational10.setPosition(rotationEnd10);
            robot.servoRotational25.setPosition(rotationEnd25);
        }
        if(gamepad1.x){
            robot.servoRotational10.setPosition(rotationStart10);
            robot.servoRotational25.setPosition(rotationStart25);
        }
        if(gamepad1.dpad_down){
            robot.continuousServo.setPosition(endContinuous);
        }
        else if(gamepad1.dpad_up){
            robot.continuousServo.setPosition(startContinuous);
        }
        else{
            robot.continuousServo.setPosition(0.5);
        }
//TODO make a limit so it doesn't smack the bottom of it
        if (debug){
            if(gamepad1.a){
                robot.loaderServo.setPosition(loaderServoSetPosition+=0.001);
            }

        }

//       left = -gamepad1.left_stick_y;
//       right = -gamepad1.right_stick_y;
//
//        robot.leftDrive.setPower(left);
//        robot.rightDrive.setPower(right);
//
//        telemetry.addData("left",  "%.2f", left);
//        telemetry.addData("right", "%.2f", right);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}




