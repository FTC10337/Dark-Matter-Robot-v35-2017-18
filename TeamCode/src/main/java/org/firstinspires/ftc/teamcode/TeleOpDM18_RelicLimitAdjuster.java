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

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

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

@TeleOp(name="Relic Adjuster", group="DM18")
//@Disabled
public class TeleOpDM18_RelicLimitAdjuster extends OpMode {

    /* Declare OpMode members. */
    HardwareDM18 robot = new HardwareDM18(); // use the class created to define a Pushbot's hardware
    // could also use HardwarePushbotMatrix class.


    boolean intake = false;
    boolean resetLift = false;
    boolean resetLiftTop = false;
    boolean autoLift = false;
    boolean plusCurState = false;
    boolean minusCurState = false;
    boolean isButtonPressed = false;
    double intakeDistance = 8.0;
    int curState = 0;
    int lastState = 0;
    double Pos = 0;
    double Pivot = 0.0;
    double Claw = 0.0;

    // Storage for reading adaFruit color sensor for beacon sensing
    // adaHSV is an array that will hold the hue, saturation, and value information.
    float[] adaHSV = {0F, 0F, 0F};

    // adaValues is a reference to the adaHSV array.
    final float adaValues[] = adaHSV;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap, true, true);



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
    @Override
    public void start() {
        robot.gripper.setBothOpen();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {


        telemetry.update();
        telemetry.addData("Relic EncPos: ", robot.relic.relicMotor.getCurrentPosition());
        telemetry.addData("Pivot: ", robot.relic.relicPivot.getPosition());
        telemetry.addData("Claw: ", robot.relic.relicGrip.getPosition());
        telemetry.addData("Extension inches: ", robot.relic.getExtensionDistanceInches());

        if (gamepad2.left_stick_y > 0.2 || gamepad2.left_stick_y < -0.2) {
            robot.relic.relicMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            double relicPower = -gamepad2.left_stick_y;
            relicPower = smoothPowerCurve(deadzone(relicPower, 0.20));
            // Driver 2 control to slow down extension speed for fine adjustments
            if (gamepad2.right_bumper || gamepad2.left_bumper) {
                relicPower = relicPower / 2;
            }

            if (robot.relic.relicMotor.getCurrentPosition() >= 0) {
                relicPower = Range.clip(relicPower, -1, 1);
            } else {
                relicPower = Range.clip(relicPower, 0, 1);
            }
            robot.relic.relicMotor.setPower(relicPower);
        } else robot.relic.stopRelicExtension();

        if (gamepad2.right_stick_y > 0.2 || gamepad2.right_stick_y < -0.2) {
            double speed = smoothPowerCurve(deadzone(-gamepad2.right_stick_y, 0.2));
            speed = (speed / (1 + robot.relic.relicPivot.getPosition()));
            if (gamepad2.left_bumper || gamepad2.right_bumper) {
                speed = speed / 8;
            }
            speed = Range.clip(speed, -1, 1);
            robot.relic.rotate(-speed);
        }

        Pivot = robot.relic.relicPivot.getPosition();
        Claw = robot.relic.relicGrip.getPosition();

        if (gamepad2.y) {
            Pivot += 0.0001;
        } else if (gamepad2.a) {
            Pivot -= 0.0001;
        }

        Pos = Range.clip(Pos, 0, 1);
        robot.relic.relicPivot.setPosition(Pivot);

        if (gamepad2.right_trigger > 0.2) {
            Claw += 0.001;
        } else if (gamepad2.left_trigger > 0.2) {
            Claw -= 0.001;
        }

        Claw = Range.clip(Claw, 0, 1);
        robot.relic.relicGrip.setPosition(Claw);

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop () {
    }

    /**
     * This does the cubic smoothing equation on joystick value.
     * Assumes you have already done any deadzone processing.
     *
     * @param x  joystick input
     * @return  smoothed value
     */
    protected double smoothPowerCurve (double x) {
        //double a = this.getThrottle();
        double a = 1.0;         // Hard code to max smoothing
        double b = 0.05;      // Min power to overcome motor stall

        if (x > 0.0)
            return (b + (1.0-b)*(a*x*x*x+(1.0-a)*x));

        else if (x<0.0)
            return (-b + (1.0-b)*(a*x*x*x+(1.0-a)*x));
        else return 0.0;
    }

    /**
     * Add deadzone to a stick value
     *
     * @param rawStick  Raw value from joystick read -1.0 to 1.0
     * @param dz   Deadzone value to use 0 to 0.999
     * @return    Value after deadzone processing
     */
    protected double deadzone(double rawStick, double dz) {
        double stick;

        // Force limit to -1.0 to 1.0
        if (rawStick > 1.0) {
            stick = 1.0;
        } else if (rawStick < -1.0) {
            stick = -1.0;
        } else {
            stick = rawStick;
        }

        // Check if value is inside the dead zone
        if (stick >= 0.0){
            if (Math.abs(stick) >= dz)
                return (stick - dz)/(1 -  dz);
            else
                return 0.0;

        }
        else {
            if (Math.abs(stick) >= dz)
                return (stick + dz)/(1 - dz);
            else
                return 0.0;

        }
    }


// Intake Control

    public void intakeControl()
    {

        // Give the intake a chance to adjust speeds in cycle
        robot.intake.updateInPower();   // Must be called each cycle for speed to vary properly

        // Intake IN
        if (gamepad1.right_trigger > 0.5) {
            robot.intake.setIn();
            intake = true;
        }

        // Intake STOP
        if (intake && robot.intake.detectGlyph()) {
            robot.intake.setStop();
            intake = false;
        }

        // Intake OUT
        if (gamepad1.left_trigger > 0.5) {
            robot.intake.setOut();
            intake = false;
        }

        // Intake STOP
        if (gamepad1.a) {
            robot.intake.setStop();
            intake = false;
        }
        // Intake open & close
        if (gamepad1.x) {
            robot.intake.setClosed();
        }
        if (gamepad1.b) {
            robot.intake.setOpen();

        }

    }


}


