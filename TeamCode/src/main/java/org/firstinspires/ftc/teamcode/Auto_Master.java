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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;


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

public abstract class Auto_Master extends LinearOpMode {

    HardwareDM18         robot   = new HardwareDM18 ();   // Use a Pushbot's hardware
    AutoHelper auto = new AutoHelper();

    RelicRecoveryVuMark vuMark;

    int angleAdjust = 25;

    int left1Pos;
    int left2Pos;
    int right1Pos;
    int right2Pos;


    /**
     * The main routine of the OpMode.
     *
     * @throws InterruptedException
     */
    @Override
    public void runOpMode() throws InterruptedException {

        // Init the robot hardware -- including gyro and range finder
        robot.init(hardwareMap, true, true);

        // Start up all the common Auto stuff

        auto.init(robot, hardwareMap, this, iAmBlue());

        robot.intake.setStop();
        // Set a flipTimer of how often to update gyro status telemetry
        ElapsedTime updateGyroStartTimer = new ElapsedTime();
        updateGyroStartTimer.reset();

        while (!isStarted()) {
            if (updateGyroStartTimer.milliseconds() >= 500) {
                // Update telemetry every 0.5 seconds
                telemetry.addData("IMU calibrated: ", robot.adaGyro.isSystemCalibrated());
                telemetry.addData("IMU Gyro calibrated:  ", robot.adaGyro.isGyroCalibrated());

                // Do a gyro read to keep it "fresh"
                telemetry.addData("Gyro heading: ", auto.readGyro());
                telemetry.update();

                // And reset the flipTimer
                updateGyroStartTimer.reset();
            }
            idle();
        }

        telemetry.clearAll();

        // Startup vuforia and other start of auto tasks
        auto.processStartButton();

        RobotLog.i("DM10337- Auto Pressed Start");
        // Step through each leg of the path,

        // Make sure the gyro is zeroed
        auto.zeroGyro();

        RobotLog.i("DM10337 - Gyro bias set to " + auto.headingBias);

        robot.lift.resetFloorPos();

        robot.relic.setRelicPivotGrabPos();

        sleep (200);

        robot.relic.setRelicPivotKickstand();

     // Main robot auto sequence

        auto.processJewel();

        readVuMark();

        driveToBox();

        driveToPile();

        loadFirstGlyph();

        if (auto.glyphsCollected == 1){

            collectSecondGlyph();

            loadSecondGlyph();

        } else {

            auto.collectGlyph(0.3, 10, 3, true, -90);
            loadFirstGlyph();
        }

        returnToBox();

        placeExtraGlyphs();

        park();

        readyForRelic();

        RobotLog.i("DM10337- Finished last move of auto");


        telemetry.addData("Path", "Complete");
        telemetry.update();
    }


    public void readVuMark() {
        vuMark = RelicRecoveryVuMark.from(auto.relicTemplate);

        telemetry.addData("VuMark", "%s visible", vuMark);

    }


    public abstract void driveToBox() throws InterruptedException;

    public abstract void park() throws InterruptedException;


    public void driveToPile() throws InterruptedException {
    }

    public void loadFirstGlyph() throws InterruptedException {

    }

    public void collectSecondGlyph() throws InterruptedException{

    }

    public void loadSecondGlyph() throws InterruptedException {

    }

    public void returnToBox() throws InterruptedException {
    }

    public void placeExtraGlyphs() throws InterruptedException {
    }

    public abstract boolean iAmBlue();

    public void readyForRelic() {
        robot.relic.setRelicPivotGrabPos();
        sleep(500);
        robot.relic.setRelicPivotKickstand();
        robot.relic.setRelicGripOpen();
    }


}
