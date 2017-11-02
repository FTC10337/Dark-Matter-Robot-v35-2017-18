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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This file illustrates the concept of driving up to a line and then stopping.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code shows using two different light sensors:
 *   The Primary sensor shown in this code is a legacy NXT Light sensor (called "sensor_light")
 *   Alternative "commented out" code uses a MR Optical Distance Sensor (called "sensor_ods")
 *   instead of the LEGO sensor.  Chose to use one sensor or the other.
 *
 *   Setting the correct WHITE_THRESHOLD value is key to stopping correctly.
 *   This should be set half way between the light and dark values.
 *   These values can be read on the screen once the OpMode has been INIT, but before it is STARTED.
 *   Move the senso on asnd off the white line and not the min and max readings.
 *   Edit this code to make WHITE_THRESHOLD half way between the min and max.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Jewel BlueTeam", group="DM18")
@Disabled
public class Matthew_Pakorn_autoJewelBlue extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareDM18         robot   = new HardwareDM18();   // Use a Pushbot's hardware

    private ElapsedTime     runtime = new ElapsedTime();

    public void runOpMode()  throws InterruptedException{
        robot.init(hardwareMap, true);

        // Wait for the game to start (driver presses PLAY)
        // Abort this loop is started or stopped.
        while (!(isStarted() || isStopRequested())) {


            idle();
        }

       //Deploy the jewel arm
        //robot.jewelServo.setPosition(robot.JEWEL_DEPLOY);
        double armPos = robot.jewelServo.getPosition();
        double armIncr = (robot.JEWEL_DEPLOY - armPos)/50;
        while (armPos < robot.JEWEL_DEPLOY) {
            armPos += armIncr;
            robot.jewelServo.setPosition(armPos);
            sleep(20);
        }

        robot.jewelCS.enableLed(true);
        sleep(3000);

        // Check if we see blue or red
        if (robot.jewelCS.red() >= 1) {
            // We see red
            robot.jewelRotServo.setPosition(iAmBlue()?robot.JEWEL_ROT_REV:robot.JEWEL_ROT_FWD);
            sleep(1000);
        } else if (robot.jewelCS.blue() >= 1) {
            // We see blue
            robot.jewelRotServo.setPosition(iAmBlue()?robot.JEWEL_ROT_FWD:robot.JEWEL_ROT_REV);
            sleep(1000);
        } else {
            // We saw neither blue nor red so do nothing
            sleep(1000);
        }

        // Reset jewel arm
        robot.jewelRotServo.setPosition(robot.JEWEL_ROT_HOME);
        armPos = robot.jewelServo.getPosition();
        armIncr = (robot.JEWEL_HOME - armPos)/50;
        while (armPos > robot.JEWEL_HOME) {
            armPos += armIncr;
            robot.jewelServo.setPosition(armPos);
            sleep(20);
        }
        //robot.jewelServo.setPosition(robot.JEWEL_HOME);
        sleep(2000);
        robot.jewelCS.enableLed(false);


        encoderDrive(0.5, 36.0, 5.0, false, 0.0);


    }

    public boolean iAmBlue() {
        return true;
    }

    /**
     * Abbreviated call to encoderDrive w/o range aggressive turning or finding adjustments
     *
     * @param speed
     * @param distance
     * @param timeout
     * @param useGyro
     * @param heading
     * @throws InterruptedException
     */
    public void encoderDrive(double speed,
                             double distance,
                             double timeout,
                             boolean useGyro,
                             double heading) throws InterruptedException {
        encoderDrive(speed, distance, timeout, useGyro, heading, false, false, 0.0);
    }


    /**
     * Abbreviated call to encoderDrive w/o range finding adjustments
     *
     * @param speed
     * @param distance
     * @param timeout
     * @param useGyro
     * @param heading
     * @param aggressive
     * @throws InterruptedException
     */
    public void encoderDrive(double speed,
                             double distance,
                             double timeout,
                             boolean useGyro,
                             double heading,
                             boolean aggressive) throws InterruptedException {
        encoderDrive(speed, distance, timeout, useGyro, heading, aggressive, false, 0.0);
    }

    /**
     *
     * Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     *
     * @param speed                 Motor power (0 to 1.0)
     * @param distance              Inches
     * @param timeout               Seconds
     * @param useGyro               Use gyro to keep/curve to an absolute heading
     * @param heading               Heading to use
     * @throws InterruptedException
     */
    public void encoderDrive(double speed,
                             double distance,
                             double timeout,
                             boolean useGyro,
                             double heading,
                             boolean aggressive,
                             boolean userange,
                             double maintainRange) throws InterruptedException {

        // Calculated encoder targets
        int newL1Target;
        int newR1Target;
        int newL2Target;
        int newR2Target;

        // The potentially adjusted current target heading
        double curHeading = heading;

        // Speed ramp on start of move to avoid wheel slip
        final double MINSPEED = 0.30;           // Start at this power
        final double SPEEDINCR = 0.015;         // And increment by this much each cycle
        double curSpeed;                        // Keep track of speed as we ramp

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            //RobotLog.i("DM10337- Starting encoderDrive speed:" + speed +
            //        "  distance:" + distance + "  timeout:" + timeout +
            //        "  useGyro:" + useGyro + " heading:" + heading + "  maintainRange: " + maintainRange);

            // Calculate "adjusted" distance  for each side to account for requested turn during run
            // Purpose of code is to have PIDs closer to finishing even on curved moves
            // This prevents jerk to one side at stop
            double leftDistance = distance;
            double rightDistance = distance;
            /*if (useGyro) {
                // We are gyro steering -- are we requesting a turn while driving?
                double headingChange = getError(curHeading) * Math.signum(distance);
                if (Math.abs(headingChange) > 5.0) {
                    //Heading change is significant enough to account for
                    if (headingChange > 0.0) {
                        // Assume 16 inch wheelbase
                        // Add extra distance to the wheel on outside of turn
                        rightDistance += Math.signum(distance) * 2 * 3.1415 * 16 * headingChange / 360.0;
                        //RobotLog.i("DM10337 -- Turn adjusted R distance:" + rightDistance);
                    } else {
                        // Assume 16 inch wheelbase
                        // Add extra distance from the wheel on inside of turn
                        // headingChange is - so this is increasing the left distance
                        leftDistance -= Math.signum(distance) * 2 * 3.1415 * 16 * headingChange / 360.0;
                        //RobotLog.i("DM10337 -- Turn adjusted L distance:" + leftDistance);
                    }
                }
            }*/

            // Determine new target encoder positions, and pass to motor controller
            newL1Target = robot.leftDrive1.getCurrentPosition() + (int)(leftDistance * robot.COUNTS_PER_INCH);
            newL2Target = robot.leftDrive2.getCurrentPosition() + (int)(leftDistance * robot.COUNTS_PER_INCH);
            newR1Target = robot.rightDrive1.getCurrentPosition() + (int)(rightDistance * robot.COUNTS_PER_INCH);
            newR2Target = robot.rightDrive2.getCurrentPosition() + (int)(rightDistance * robot.COUNTS_PER_INCH);

            while(robot.leftDrive1.getTargetPosition() != newL1Target){
                robot.leftDrive1.setTargetPosition(newL1Target);
                sleep(1);
            }
            while(robot.rightDrive1.getTargetPosition() != newR1Target){
                robot.rightDrive1.setTargetPosition(newR1Target);
                sleep(1);
            }
            while(robot.leftDrive2.getTargetPosition() != newL2Target){
                robot.leftDrive2.setTargetPosition(newL2Target);
                sleep(1);
            }
            while(robot.rightDrive2.getTargetPosition() != newR2Target){
                robot.rightDrive2.setTargetPosition(newR2Target);
                sleep(1);
            }

            // Turn On motors to RUN_TO_POSITION
            robot.setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();

            speed = Math.abs(speed);    // Make sure its positive
            curSpeed = Math.min(MINSPEED,speed);

            // Set the motors to the starting power
            robot.leftDrive1.setPower(Math.abs(curSpeed));
            robot.rightDrive1.setPower(Math.abs(curSpeed));
            robot.leftDrive2.setPower(Math.abs(curSpeed));
            robot.rightDrive2.setPower(Math.abs(curSpeed));

            // keep looping while we are still active, and there is time left, until at least 1 motor reaches target
            while (opModeIsActive() &&
                    (runtime.seconds() < timeout) &&
                    robot.leftDrive1.isBusy() &&
                    robot.leftDrive2.isBusy() &&
                    robot.rightDrive1.isBusy() &&
                    robot.rightDrive2.isBusy()) {

                // Ramp up motor powers as needed
                if (curSpeed < speed) {
                    curSpeed += SPEEDINCR;
                }
                double leftSpeed = curSpeed;
                double rightSpeed = curSpeed;

                /*
                // Doing gyro heading correction?
                if (useGyro){

                    // Get the difference in distance from wall to desired distance
                    double errorRange = robot.rangeSensor.getDistance(DistanceUnit.CM) - maintainRange;

                    if (userange) {
                        if (Math.abs(errorRange) >= RANGE_THRESHOLD) {
                            // We need to course correct to right distance from wall
                            // Have to adjust sign based on heading forward or backward
                            curHeading = heading - Math.signum(distance) * errorRange * P_DRIVE_COEFF_3;
                            RobotLog.i("DM10337 - Range adjust -- range:" + errorRange + "  heading: " + curHeading + "  actual heading: " + readGyro());
                        } else {
                            // We are in the right range zone so just use the desired heading w/ no adjustment
                            curHeading = heading;
                        }
                    }

                    // adjust relative speed based on heading
                    double error = getError(curHeading);
                    double steer = getSteer(error,
                            (aggressive?P_DRIVE_COEFF_1:P_DRIVE_COEFF_2));

                    // if driving in reverse, the motor correction also needs to be reversed
                    if (distance < 0)
                        steer *= -1.0;

                    // Adjust motor powers for heading correction
                    leftSpeed -= steer;
                    rightSpeed += steer;

                    // Normalize speeds if any one exceeds +/- 1.0;
                    double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                    if (max > 1.0)
                    {
                        leftSpeed /= max;
                        rightSpeed /= max;
                    }

                }
                */

                // And rewrite the motor speeds
                robot.leftDrive1.setPower(Math.abs(leftSpeed));
                robot.rightDrive1.setPower(Math.abs(rightSpeed));
                robot.leftDrive2.setPower(Math.abs(leftSpeed));
                robot.rightDrive2.setPower(Math.abs(rightSpeed));

                // Allow time for other processes to run.
                idle();
            }


            // RobotLog.i("DM10337- encoderDrive done" +
            //        "  lftarget: " +newLFTarget + "  lfactual:" + robot.lfDrive.getCurrentPosition() +
            //        "  lrtarget: " +newLRTarget + "  lractual:" + robot.lrDrive.getCurrentPosition() +
            //        "  rftarget: " +newRFTarget + "  rfactual:" + robot.rfDrive.getCurrentPosition() +
            //        "  rrtarget: " +newRRTarget + "  rractual:" + robot.rrDrive.getCurrentPosition() +
            //        "  heading:" + readGyro());

            // Stop all motion;
            robot.leftDrive1.setPower(0);
            robot.rightDrive1.setPower(0);
            robot.leftDrive2.setPower(0);
            robot.rightDrive2.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }
}
