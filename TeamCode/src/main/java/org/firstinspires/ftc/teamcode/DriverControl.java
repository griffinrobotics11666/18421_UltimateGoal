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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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

@TeleOp(name="18421 Blaster", group="UltimateGoalBot")
//@Disabled
public class DriverControl extends OpMode{

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
    final double loaderServoSetPosition = 0.0;
    final double loaderServoMaxPosition = 0.25;
    final double shooterMotorMinPower = 0.0;
    // i hate fianl
    final double shooterMotorMidPower = 0.75;
    final double shooterMotorMaxPower = 1.0; // eazy
    final double startContinuous = 0.0;
    final double endContinuous = 1.0;
    final double rotationStart50 = 0;
    final  double rotationEnd50 = 0.5;
    final double rotationStart25 = 0;
    final  double rotationEnd25 = 0.25;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        telemetry.addData("Status" , "Initialized");
        //robot.init(Hardwarebot2); do we need this???????

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
            robot.shooter.setPower(0.75);
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
            robot.loaderServo.setPosition(0.25);

        } else {
            robot.loaderServo.setPosition(0.0);
        }
        if (gamepad1.a){
            robot.servoRotational50.setPosition(rotationEnd50);
            robot.servoRotational25.setPosition(rotationEnd25);
        }
        if(gamepad1.x){
            robot.servoRotational50.setPosition(rotationStart50);
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

//
//        left = -gamepad1.left_stick_y;
//        right = -gamepad1.right_stick_y;
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
