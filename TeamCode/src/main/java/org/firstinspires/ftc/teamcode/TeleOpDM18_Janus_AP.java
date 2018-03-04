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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
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

@TeleOp(name="Teleop Janus AP", group="DM18")
//@Disabled
public class TeleOpDM18_Janus_AP extends OpMode {

    /* Declare OpMode members. */
    HardwareDM18 robot = new HardwareDM18(); // use the class created to define a Pushbot's hardware
    // could also use HardwarePushbotMatrix class.


    public enum States {INIT_1, INIT_2, INIT_3, RESET_1, RESET_2, RESET_2_1,  RESET_3, RESET_4, AUTO_LOAD_INIT, AUTO_LOAD_1, AUTO_LOAD_1_2, AUTO_LOAD_2, AUTO_LOAD_3, AUTO_LOAD_4, AUTO_LOAD_5, AUTO_LOAD_SECOND}

    States nStates = States.INIT_1;

    boolean startInit = false;
    boolean init_TeleOp = true;
    boolean init_AutoLoad = false;
    boolean init_Reset = false;

    boolean autoPark = false;
    boolean autoParking = false;

    boolean glyphBump = false;

    boolean glyphMode = true;
    boolean relicMode = false;
    boolean firstModeChange = true;

    boolean reverseDrive = false;

    boolean autoMove = false;

    boolean resetLiftBtm = false;
    boolean resetLiftTop = false;
    boolean liftChangePos = false;

    boolean topGripisClosed = false;

    boolean isButtonPressed = false;

    boolean flip = false;

    boolean glyphDetected = false;

    boolean slowDriveTrain = false;
    boolean slowDriveTrain2 = false;
    boolean slowDriveTrainOveride = false;

    boolean squaringGlyph = false;
    boolean timedStopIntake = false;

    int turnCoefficient = 1;
    int driveCoefficient = 1;

    int liftFloorTarget = 0;

    double liftPower = 0;

    ElapsedTime intakeStopTimer = new ElapsedTime();
    ElapsedTime parkTimer = new ElapsedTime();

    Orientation angles;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap, false, false);

        robot.relic.setRelicPivotKickstand();
        robot.relic.setRelicGripOpen();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");


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


        telemetry.addData("Relic Mode: ", relicMode);

        //telemetry.addData("alpha: " + robot.intake.glyphColorSensor.alpha(), "dist: " + robot.intake.intakeDistance);
        //telemetry.addData("LEFT_AVG: ", robot.intake.distSensor_leftAvg.average());
        //telemetry.addData("RIGHT_AVG: ", robot.intake.distSensor_rightAvg.average());
        //telemetry.addData("LEFT: ", robot.intake.distanceSensor_left.getDistance(DistanceUnit.CM));
        //telemetry.addData("RIGHT: ", robot.intake.distanceSensor_right.getDistance(DistanceUnit.CM));
        //telemetry.addData("timer: "+ intakeStopTimer.milliseconds() +
        //        "sqg: " + squaringGlyph +"  tsi:  ", timedStopIntake);

        telemetry.update();

        /*
        INITIATION SEQUENCE. RESETS LIFT AND GRIPPER
         */
        if (init_TeleOp) { // AUTO SEQUENCED STATES - INIT, LOAD & RESET
            switch (nStates) {
                case INIT_1: // INIT DRIVER CONTROL _ RESET LIFT TO TOP AND INIT GRIPPER
                    if (gamepad1.left_stick_y > 0.2 || gamepad1.left_stick_y < -0.2) {
                        startInit = true;
                    }

                    if (startInit) {
                        if (robot.lift.resetTopPos()) {
                            // initiates gripper. Init will set grippers closed, flipped in correct orientation, and pusher in home position.
                            robot.gripper.init(hardwareMap, "gripP", "gripB", "gripRotate", "gripExtend");
                            robot.gripper.setBothOpen();
                            robot.gripper.setExtendIn();
                            nStates = States.INIT_2;
                        }
                    }

                    break;

                case INIT_2: // Wait for the init moves to finish before we lower down

                    if (!robot.gripper.isFlipping()) {
                        nStates = States.INIT_3;
                        robot.lift.setLiftBtm();
                    }
                    break;

                case INIT_3: // INIT DRIVER CONTROL _ LIFT TO BOTTOM POSITION
                    telemetry.addData("Enc: ", robot.lift.liftMotor.getCurrentPosition());
                    if (robot.lift.reachedFloor()) {
                        if (robot.lift.resetFloorPos()) {
                            init_TeleOp = false;
                        }
                    }
                    break;
            }

        }

        /*
          DRIVE 1 CONTROLS
         */

        if (gamepad1.y && !isButtonPressed) {
            isButtonPressed = true;
            autoParking = true;
            autoPark(0.6, -20.0);
            parkTimer.reset();
            RobotLog.i("DM10337 -- AUTO PARKING ENGAGED!");
        }

        // Override auto park
        if (((autoParking) && (gamepad1.left_stick_y > 0.2 || gamepad1.left_stick_y < -0.2 || gamepad1.right_stick_x > 0.2 || gamepad1.right_stick_x < -0.2)) || parkTimer.seconds() > 30.0) {
            autoParking = false;
            robot.setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
         }

        // Intake controls. Some intake controls do not function while auto load sequence is activate
        intakeControl();

        if (!autoParking){
            double left;
            double right;

            double throttle = -gamepad1.left_stick_y;
            double direction = gamepad1.right_stick_x;

            if (reverseDrive) {
                throttle = -throttle;
            }

            // Toggle drive train speed to SLOW when pusher is OUT or RELIC is extended
            if (!init_TeleOp && !init_Reset && robot.gripper.isPusherOut()) {
                slowDriveTrain = true;
                slowDriveTrainOveride = true;
                robot.intake.setOpen();
                if (robot.intake.isIntakeInOn){
                     robot.intake.setStop();
                }
            } else slowDriveTrain = false;

            // Driver 1 ability to slow down drivetrain
            if (gamepad1.left_bumper && !init_AutoLoad && !init_Reset) {
                slowDriveTrain2 = true;
                slowDriveTrainOveride = true;
            } else slowDriveTrain2 = false;

            // Change drive and turn speed coefficient
            if (!init_TeleOp && (slowDriveTrain || slowDriveTrain2) && slowDriveTrainOveride) {
                turnCoefficient = 4;
                driveCoefficient = 2;
            } else {
                turnCoefficient = 2;
                driveCoefficient = 1;
            }

            // Smooth and deadzone the joystick values
            throttle = smoothPowerCurve(deadzone(throttle, 0.10)) / driveCoefficient;
            direction = (smoothPowerCurve(deadzone(direction, 0.10))) / turnCoefficient;

            // Calculate the drive motors for left and right
            right = throttle - direction;
            left = throttle + direction;

            // clip the right/left values so that the values never exceed +/- 1
            right = Range.clip(right, -1, 1);
            left = Range.clip(left, -1, 1);
            // Normalize speeds if any one exceeds +/- 1.0;
            double max = Math.max(Math.abs(right), Math.abs(left));
            if (max > 1.0) {
                left /= max;
                right /= max;
            }

            // Apply power to drive motors
            robot.leftDrive1.setPower(left);
            robot.leftDrive2.setPower(left);
            robot.rightDrive1.setPower(right);
            robot.rightDrive2.setPower(right);
        }

        // Determine if glyph is detected in intake. Necessary to start auto-load sequence
        glyphDetected = robot.intake.detectGlyph();


        /*
          DRIVER_2 CONTROLS
         */

        // Determine if button is still pressed. Used to prevent multiple firing of robot actions when button is held down.
        if (!gamepad2.a && !gamepad2.x && !gamepad2.y && !gamepad2.b && !gamepad2.left_bumper && !gamepad2.right_bumper && !gamepad2.dpad_down
                && !gamepad2.dpad_left && !gamepad2.dpad_right && !gamepad2.dpad_up && !gamepad2.start) {
            isButtonPressed = false;
        }

        /*
         GLYPH MODE CONTROLS for DRIVER 2
          */

        // Driver 2 can switch between glyph mode and relic mode by pressing "dpad_down" and "b" simultaneously

        if (glyphMode) {

            // SETS DRIVER 2 to RELIC MODE
            if (gamepad2.start && gamepad2.y && !isButtonPressed) {

                RobotLog.i("DM10337 -- SWITCHING to RELIC mode");

                isButtonPressed = true;

                // Override auto moves
                autoMove = false;
                init_AutoLoad = false;
                init_Reset = false;

                // reset all auto lift booleans to default
                resetLiftBtm = false;
                resetLiftTop = false;
                liftChangePos = false;
                flip = false;

                // set DRIVE MODE to RELIC
                relicMode = true;
                glyphMode = false;

                if (firstModeChange) {
                    robot.relic.setRelicGripOpen();
                    firstModeChange = false;
                }
            }

            /*
            MANUAL LIFT CONTROLS - DRIVER 2
            */

            // / MANUAL MOVEMENT OF LIFT _ Moving right stick stops all auto movements
            if (gamepad2.right_stick_y > 0.2 || gamepad2.right_stick_y < -0.2) {

                if (init_TeleOp) {
                    robot.gripper.init(hardwareMap, "gripP", "gripB", "gripRotate", "gripExtend");
                    if (!robot.intake.detectGlyph()) robot.gripper.setBothOpen();
                    init_TeleOp = false;
                }

                // Override auto moves
                autoMove = false;
                init_AutoLoad = false;
                init_Reset = false;

                // reset all auto lift booleans to default
                resetLiftBtm = false;
                resetLiftTop = false;
                liftChangePos = false;
                flip = false;

                if (robot.lift.distFromBottom() < 8.0){
                    // Open intake wheels
                    robot.intake.setOpen();
                    robot.intake.setStop();
                }

                // Move lift
                robot.lift.liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                liftPower = -gamepad2.right_stick_y;
                liftPower = smoothPowerCurve(deadzone(liftPower, 0.20));
                if (!robot.lift.liftLimitT.getState()) {
                    liftPower = Range.clip(liftPower, -1, 0);
                } else if (!robot.lift.liftLimitB.getState()) {
                    liftPower = Range.clip(liftPower, 0, 1);
                } else liftPower = Range.clip(liftPower, -1, 1);

                robot.lift.liftMotor.setPower(liftPower);

            } else if (!autoMove && !init_TeleOp) {
                // Stops lift if no joystick input or auto moves active
                robot.lift.stopLift();
            }

            // SETS LIFT POSITION
            if (gamepad2.dpad_up && !isButtonPressed) {
                isButtonPressed = true;
                liftFloorTarget = 2;            // Top floor
                liftChangePos = true;
                autoMove = true;
            }

            if ((gamepad2.dpad_left || gamepad2.dpad_right) && !isButtonPressed) {
                isButtonPressed = true;
                liftFloorTarget = 1;            // Middle floor
                liftChangePos = true;
                autoMove = true;
            }

            if (gamepad2.dpad_down && !gamepad2.b && !isButtonPressed) {
                isButtonPressed = true;
                liftFloorTarget = 0;            // Bottom floor
                liftChangePos = true;
                autoMove = true;
            }

            // SETS LIFT POSITION AND MOVES LIFT
            if (liftChangePos && autoMove) {
                switch (liftFloorTarget) {
                    case 0:
                        robot.lift.setLiftBtm();
                        liftChangePos = false;
                        resetLiftBtm = true;
                        break;
                    case 1:
                        robot.lift.setLiftMid();
                        liftChangePos = false;
                        break;
                    case 2:
                        robot.lift.setLiftTop();
                        liftChangePos = false;
                        resetLiftTop = true;
                        break;
                }
            }

            // Find BTM limit switch after changing lift position BTM
            if (resetLiftBtm && robot.lift.reachedFloor() && autoMove) {
                if (robot.lift.resetFloorPos()) {
                    resetLiftBtm = false;
                }
            }

            // Find TOP limit switch after changing lift position to TOP
            if (resetLiftTop && robot.lift.reachedFloor() && autoMove) {
                if (robot.lift.resetTopPos()) {
                    resetLiftTop = false;
                }
            }

        /*
          GRIPPER CONTROLS - DRIVER 2
         */

            // INITIATE FLIP
            if (gamepad2.x && !isButtonPressed && !init_TeleOp && !init_AutoLoad) {
                isButtonPressed = true;
                flip = true;
                autoMove = true;
            }

            // FLIP GRIPPER
            if (flip && autoMove && (robot.lift.distFromBottom() >= 7.75) && !init_AutoLoad) {
                // Flip after determining gripper is high enough
                robot.gripper.flip();
                flip = false;
            } else if ((flip && autoMove && robot.lift.distFromBottom() < 7.75) && !init_AutoLoad) {
                // Move gripper to top position before flipping if in another lift position
                robot.lift.setLiftHeight(8.25);
            }

            // PUSHER IN/OUT
            if (gamepad2.left_stick_y > 0.2 && !init_TeleOp && !init_Reset || gamepad2.left_stick_y < -0.2 && !init_TeleOp && !init_Reset) {

                // reset all auto lift booleans to default
                resetLiftBtm = false;
                resetLiftTop = false;
                liftChangePos = false;
                flip = false;


                if ((robot.lift.distFromBottom() > 8) ||
                        (!robot.intake.isClosed() && !robot.intake.isMoving() &&
                                (robot.gripper.isBtmClosed() || robot.gripper.isBtmPartialOpen()) && !robot.gripper.isMoving())) {
                    double speed = smoothPowerCurve(deadzone(-gamepad2.left_stick_y, 0.2));
                    speed = Range.clip(speed, -1, 1);
                    robot.gripper.moveInOut(speed);
                } else {
                    robot.intake.setOpen();
                    if (robot.gripper.isBtmOpen()) {
                        robot.gripper.setBtmClosed();

                    }
                }
            }


            // OPEN & CLOSE GRIPPERS
            // close grippers
            if ((gamepad2.right_trigger > 0.5) && !init_TeleOp && !init_AutoLoad && !init_Reset) {
                robot.gripper.setBtmClosed();
            }
            if (gamepad2.right_bumper && !isButtonPressed && !init_TeleOp && !init_AutoLoad && !init_Reset) {
                robot.gripper.setTopClosed();
                isButtonPressed = true;
                topGripisClosed = true;
            }

            // open grippers
            if ((gamepad2.left_trigger > 0.5) && !init_TeleOp && !init_AutoLoad && !init_Reset) {
                // Set fully open
                robot.gripper.setBtmOpen();

            }
            if (gamepad2.left_bumper && !isButtonPressed && !init_TeleOp && !init_AutoLoad && !init_Reset) {
                // Set fully open
                robot.gripper.setTopOpen();
                topGripisClosed = false;
                isButtonPressed = true;
            }

        /*
        AUTO RESET LIFT/GRIPPER SEQUENCE
         */

            // RESET auto load sequence. To be used by both drivers is something in sequence goes wrong
            if (gamepad2.y && gamepad2.b && !isButtonPressed && !init_TeleOp) {
                RobotLog.i("DM10337 -- RESET glyph mechanism");
                isButtonPressed = true;
                autoMove = true;
                init_Reset = true;
                init_AutoLoad = false;

                // reset all auto lift booleans to default
                resetLiftBtm = false;
                resetLiftTop = false;
                liftChangePos = false;
                flip = false;

                nStates = States.RESET_1;
            }

            if (init_Reset && autoMove) {
                switch (nStates) {

                    case RESET_1: // OPEN INTAKE

                        slowDriveTrainOveride = false;
                        glyphBump = false;

                        if (robot.intake.isClosed()) {
                            robot.intake.setOpen();
                            robot.gripper.setBothOpen();
                            nStates = States.RESET_2;
                            RobotLog.i("DM10337 -- RESET glyph sequence 1 complete");
                        } else {
                            robot.gripper.setBothOpen();
                            nStates = States.RESET_2;
                            RobotLog.i("DM10337 -- RESET glyph sequence 1 complete");
                        }
                        break;

                    case RESET_2: // LIFT ABOVE INTAKE IF NOT ALREADY

                        if (!robot.intake.isMoving() && !robot.gripper.isReleasing()) {
                            if (robot.lift.distFromBottom() < 5.0) {
                                robot.lift.setLiftHeight(5.5);
                                nStates = States.RESET_2_1;
                                RobotLog.i("DM10337 -- RESET glyph sequence 2 complete");
                            } else {
                                nStates = States.RESET_2_1;
                                RobotLog.i("DM10337 -- RESET glyph sequence 2 complete");
                            }
                        }
                        break;

                    case RESET_2_1: // PUSHER IN & OPEN GRIPPERS

                        if (robot.lift.distFromBottom() > 5.0) {
                            robot.gripper.setExtendIn();
                            robot.gripper.setBothOpen();
                            topGripisClosed = false;
                            nStates = States.RESET_3;
                            RobotLog.i("DM10337 -- RESET glyph sequence 2_1 complete");
                        }
                        break;

                    case RESET_3:

                        if (!robot.gripper.isExtending()) {
                            robot.lift.setLiftBtm();
                            nStates = States.RESET_4;
                            RobotLog.i("DM10337 -- RESET glyph sequence 3 complete");
                        }
                        break;

                    case RESET_4:

                        if (robot.lift.reachedFloor()) {
                            if (robot.lift.resetFloorPos()) {
                                init_Reset = false;
                                RobotLog.i("DM10337 -- RESET glyph sequence 4 complete");
                            }
                        }

                }
            }

        /*
        AUTO LOAD GLYPH SEQUENCE
         */

            // INITIATE AUTO LOAD - DRIVER 2 - Can be activated when glyph is detected
            if (gamepad2.a && glyphDetected && !isButtonPressed && !init_TeleOp && !init_AutoLoad) {
                RobotLog.i("DM10337 -- Start AUTOLOAD sequence");
                isButtonPressed = true;
                nStates = States.AUTO_LOAD_INIT;
                init_AutoLoad = true;
                autoMove = true;
                init_Reset = false;
            }

            if (init_AutoLoad && autoMove) {

                glyphBump = false;

                switch (nStates) {

                    case AUTO_LOAD_INIT: // CHECK TO SEE IF LIFT IS AT BTM POS AND BTM GRIPPERS ARE OPEN
                        if (robot.gripper.isBtmClosed()) {
                            robot.gripper.setBtmOpen();
                        } else if (!robot.gripper.btmIsMoving()) {
                            robot.lift.setLiftBtm();
                        }
                        if (!robot.gripper.isBtmClosed() && robot.lift.targetPos == robot.lift.LIFT_BTM_POS && robot.lift.reachedFloor()) {
                            // Move lift down to BTM limit switch
                            if (robot.lift.resetFloorPos()) {
                                nStates = States.AUTO_LOAD_1;
                                RobotLog.i("DM10337 -- AUTO LOAD Seqeunce Init Complete");
                           }
                        }
                        break;

                    case AUTO_LOAD_1: // GRAB GLYPH & OPEN INTAKE WHEELS
                        // Grab glyph with BTM gripper
                        if (!robot.gripper.isBtmClosed()) {
                            robot.gripper.setBtmClosed();
                            nStates = States.AUTO_LOAD_1_2;
                            RobotLog.i("DM10337 -- AUTO LOAD Seqeunce 1 Complete");
                        }
                        break;

                    case AUTO_LOAD_1_2:
                        // Open intake after gripper has closed
                        if (!robot.gripper.btmIsMoving()) {
                            robot.intake.setOpen();
                            if (!topGripisClosed) {
                                nStates = States.AUTO_LOAD_2;
                                RobotLog.i("DM10337 -- AUTO LOAD Seqeunce 1_2 Complete");
                            } else {
                                nStates = States.AUTO_LOAD_SECOND;
                                RobotLog.i("DM10337 -- AUTO LOAD Seqeunce 1_2 Complete");
                            }
                        }
                        break;

                    // LOAD SEQUENCE FOR FIRST GLYPH - Determined if top gripper is OPEN, therefore does not have glyph.
                    case AUTO_LOAD_2: // LIFT TO LIFT POSITION
                        // Set lift position to move to top after intake has opened
                        if (!robot.intake.isMoving()) {
                            robot.lift.setLiftHeight(8.25);
                            nStates = States.AUTO_LOAD_3;
                            RobotLog.i("DM10337 -- AUTO LOAD Seqeunce 2 Complete");
                        }
                        break;

                    case AUTO_LOAD_3: // FLIP AFTER FLIP HEIGHT REACHED
                        if (robot.lift.distFromBottom() > 7.75) { // Check to see if lift reached height to flip
                            // If it reached target position, set intake back to closed position and start intaking again
                            robot.intake.setClosed();
                            robot.intake.setIn();
                            robot.gripper.flip(); // Flip gripper
                            topGripisClosed = true;
                            nStates = States.AUTO_LOAD_4; // Go to next state in auto load sequence
                            RobotLog.i("DM10337 -- AUTO LOAD Seqeunce 3 Complete");
                        }
                        break;

                    case AUTO_LOAD_4: // MOVE TO BTM FLOOR

                        if (!robot.gripper.isFlipping()) {
                            robot.lift.setLiftHeight(0.5); // Move lift to btm position after gripper is done flipping
                            //nStates = States.AUTO_LOAD_5;
                            init_AutoLoad = false;
                            RobotLog.i("DM10337 -- AUTO LOAD Seqeunce 4 Complete");
                        }
                        break;

                    case AUTO_LOAD_5: // RESET ENCODER TO '0' USING BTM LIMIT SWITCH
                        if (robot.lift.reachedFloor()) {
                            robot.lift.resetFloorPos();
                            init_AutoLoad = false;
                            RobotLog.i("DM10337 -- AUTO LOAD Seqeunce 5 Complete");
                        }
                        break;

                    // LOAD SEQUENCE FOR SECOND GLYPH - Determined if top gripper is CLOSED, therefore assumed it has a glyph.
                    case AUTO_LOAD_SECOND: // LIFT TO DRIVE
                        if (!robot.intake.isMoving()) {
                            robot.lift.setLiftHeight(1.0); // Lift above ground to drive
                            init_AutoLoad = false;
                            RobotLog.i("DM10337 -- AUTO LOAD Seqeunce SECOND Complete");
                        }
                        break;
                }
            }
        }

        /*
        RELIC MODE CONTROLS for DRIVER 2 //
        */

        // Driver 2 can switch between glyph mode and relic mode by pressing "dpad_down" and "b" simultaneously

        if (relicMode) {

            // SETS DRIVER 2 to GLYPH MODE
            if (gamepad2.start && gamepad2.b && !isButtonPressed) {

                isButtonPressed = true;

                // set DRIVE MODE to GLYPH
                relicMode = false;
                glyphMode = true;
                RobotLog.i("DM10337 -- SWITCHING to GLYPH mode");

            }

            // / MANUAL MOVEMENT OF RELIC EXTENSION


            if (gamepad2.left_stick_y > 0.2 || gamepad2.left_stick_y < -0.2) {
                autoMove = false;
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
            } else if (!autoMove) robot.relic.stopRelicExtension();

            // Set relic extension to out position
            if (gamepad2.dpad_up) {
                robot.relic.setRelicExtensionOut();
                autoMove = true;
            }

            // Set relic extension to in position
            if (gamepad2.dpad_down) {
                robot.relic.setRelicExtensionIn();
                autoMove = true;
            }

            // MANUAL MOVEMENT OF RELIC PIVOT
            if (gamepad2.right_stick_y > 0.2 || gamepad2.right_stick_y < -0.2) {
                double speed = smoothPowerCurve(deadzone(-gamepad2.right_stick_y, 0.2));
                speed = (speed / (1 + robot.relic.relicPivot.getPosition()));
                if (gamepad2.left_bumper || gamepad2.right_bumper) {
                    speed = speed / 8;
                }
                speed = Range.clip(speed, -1, 1);
                robot.relic.rotate(-speed);
            }

            // Relic grip grab
            if (gamepad2.a && !isButtonPressed) {
                isButtonPressed = true;
                if (robot.relic.isGripClosed()) {
                    robot.relic.setRelicGripOpen();
                } else { robot.relic.setRelicGripGrab(); }
            }

            // Relic arm to kickstand position
            if (gamepad2.b) {
                robot.relic.setRelicPivotKickstand();
                autoMove = true;
             }

            if (gamepad2.y && !isButtonPressed) {
                robot.relic.setRelicGripOpen();
                robot.relic.setRelicPivotKickstand();
                isButtonPressed = true;
            }

            // Set relic pivot to grab position and open claw
            if (gamepad2.x && robot.relic.isGripHoldingRelic()) {
                robot.relic.setRelicPivotDropPos();
            } else if (gamepad2.x && !robot.relic.isGripHoldingRelic()) {
                robot.relic.setRelicPivotGrabPos();
            }

        }
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

    public void intakeControl()  {

        // Update the distance sensor rolling averages
        robot.intake.updateDistAvg();

        // Intake IN
        if (gamepad1.right_trigger > 0.5 && !init_AutoLoad && !init_Reset) {
            robot.intake.setIn();
            robot.intake.setClosed();
            squaringGlyph = false;
            timedStopIntake = false;
        }

        // Intake IN - Start Squaring Glyph
        //if (gamepad1.right_bumper && !init_AutoLoad && !init_Reset) {
        //    squaringGlyph = true;
        //    timedStopIntake = false;
        // }

        if (gamepad1.right_bumper && !init_AutoLoad && !init_Reset) {
            robot.intake.setOpen();
            glyphBump = true;

        } else if (glyphBump && !init_AutoLoad && !init_Reset ) {
            robot.intake.setClosed();
            glyphBump = false;
        }

        // Intake SET STOP after glyph detected
        if (!timedStopIntake && robot.intake.isIntakeInOn && robot.intake.detectGlyph()) {
            squaringGlyph = true;
        }

        // Intake STOP - square glyph if necessary then stop intaking
        if (squaringGlyph) {
            // check to see if glyph is squared against backplate
            if (Math.abs(robot.intake.distLeft() - robot.intake.distRight()) > 1.0){
                // square glyph
                robot.intake.squareGlyph();
            } else {
                intakeStopTimer.reset();
                robot.intake.setIn();
                squaringGlyph = false;
                timedStopIntake = true;
            }
        }

        if (timedStopIntake && intakeStopTimer.milliseconds() > 50) {
            robot.intake.setStop();
            timedStopIntake = false;
        }


        // Intake OUT
        if (gamepad1.left_trigger > 0.5 && !init_AutoLoad && !init_Reset) {
            robot.intake.setOut();
            robot.intake.setClosed();
            timedStopIntake = false;
            squaringGlyph = false;
        }

        // Intake STOP
        if (gamepad1.a && !init_AutoLoad && !init_Reset) {
            robot.intake.setStop();
            squaringGlyph = false;
            timedStopIntake = false;
        }

        // Intake open & close
        if (gamepad1.x && !init_AutoLoad && !init_Reset) {
            robot.intake.setClosed();
        }
        if (gamepad1.b && !init_AutoLoad && !init_Reset) {
            robot.intake.setOpen();

        }
        if (gamepad1.dpad_up) {
            reverseDrive = false;
        }
        if (gamepad1.dpad_down) {
            reverseDrive = true;
        }

    }

    public void autoPark(double speed, double distance){

        double leftDistance = distance;
        double rightDistance = distance;

        // Determine new target encoder positions, and pass to motor controller
        int newLFTarget = robot.leftDrive1.getCurrentPosition() + (int)(leftDistance * robot.COUNTS_PER_INCH);
        int newLRTarget = robot.leftDrive2.getCurrentPosition() + (int)(leftDistance * robot.COUNTS_PER_INCH);
        int newRFTarget = robot.rightDrive1.getCurrentPosition() + (int)(rightDistance * robot.COUNTS_PER_INCH);
        int newRRTarget = robot.rightDrive2.getCurrentPosition() + (int)(rightDistance * robot.COUNTS_PER_INCH);

        robot.leftDrive1.setTargetPosition(newLFTarget);
        robot.rightDrive1.setTargetPosition(newRFTarget);
        robot.leftDrive2.setTargetPosition(newLRTarget);
        robot.rightDrive2.setTargetPosition(newRRTarget);

        // Turn On motors to RUN_TO_POSITION
        robot.setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set the motors to the starting power
        robot.leftDrive1.setPower(Math.abs(speed));
        robot.rightDrive1.setPower(Math.abs(speed));
        robot.leftDrive2.setPower(Math.abs(speed));
        robot.rightDrive2.setPower(Math.abs(speed));
        // set robot to move backwards
    }
    /**
     * Read the current heading direction.  Use a heading bias if we recorded one at start to account for drift during
     * the init phase of match
     *
     * @return      Current heading (Z axis)
     */
    Orientation readGyro() {
        Orientation gyroRead;

        gyroRead = robot.adaGyro.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        return gyroRead;
    }
}

