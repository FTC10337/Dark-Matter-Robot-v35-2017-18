package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by jholly on 1/4/2018.
 *
 * Exactly the same as 2G version but DON"T try to intake any extra glyphs
 */

@Autonomous(name="Auto Blue L 3G", group="DM18")
//@Disabled
public class Auto_Blue_L_3G extends Auto_Master {
    @Override
    public void driveToBox() throws InterruptedException {

        // Drive off stone
        if (iAmBlue()) {
            auto.encoderDrive(0.5, 28.0, 5.0, true, 0.0);
            auto.gyroTurn(AutoHelper.DRIVE_SPEED, -90, auto.P_TURN_COEFF);
        } else {
            auto.encoderDrive(0.5, -28.0, 5.0, true, 0.0);
            auto.gyroTurn(AutoHelper.DRIVE_SPEED, -90, auto.P_TURN_COEFF);
        }


        if (vuMark == RelicRecoveryVuMark.CENTER || vuMark == RelicRecoveryVuMark.UNKNOWN) {

            angleAdjust = 0;
            distAdjust = 0;

            // Drive forward to lineup with center cryptoglyph
            if (iAmBlue()) {
                auto.encoderDrive(AutoHelper.DRIVE_SPEED, iAmBlue()? 13.515 : 12.165, 5.0, true, -90.0);            } else {
                auto.encoderDrive(AutoHelper.DRIVE_SPEED, iAmBlue()? 13.515 : 12.165, 5.0, true, -90.0);
            }

        }
        if (vuMark == RelicRecoveryVuMark.RIGHT) {

            angleAdjust = -10;
            distAdjust = -3;

            // Drive forward to lineup with center cryptoglyph
            if (iAmBlue()) {
                auto.encoderDrive(AutoHelper.DRIVE_SPEED, iAmBlue()? 13.515 + 7.5 : 12.165 + 7.5, 5.0, true, -90.0);
            } else {
                auto.encoderDrive(AutoHelper.DRIVE_SPEED, iAmBlue()? 13.515 + 7.5 : 12.165 + 7.5, 5.0, true, -90.0);
            }

        }
        if (vuMark == RelicRecoveryVuMark.LEFT) {

            angleAdjust = 9;
            distAdjust = 5;

            // Drive forward to lineup with center cryptoglyph
            if (iAmBlue()) {
                auto.encoderDrive(AutoHelper.DRIVE_SPEED, iAmBlue()? 13.515 - 7.5 : 12.165 - 7.5, 5.0, true, -90.0);
            } else {
                auto.encoderDrive(AutoHelper.DRIVE_SPEED, iAmBlue()? 13.515 - 7.5 : 12.165 - 7.5, 5.0, true, -90.0);
            }

        }

        // Turn toward  cryptoglyph
        if (iAmBlue()) {
            auto.gyroTurn(AutoHelper.DRIVE_SPEED, 0, auto.P_TURN_COEFF);
        } else {
            auto.gyroTurn(AutoHelper.DRIVE_SPEED, 180, auto.P_TURN_COEFF);
        }

        // Outake glyph
        robot.gripper.setBothOpen();
        sleep(250);
        // Drive closer to cryptoglyph
        if (iAmBlue()) {
            auto.encoderDrive(AutoHelper.DRIVE_SPEED_SLOW, 3.0, 0.5, true, 0);
        } else {
            auto.encoderDrive(AutoHelper.DRIVE_SPEED_SLOW, 3.0, 0.5, true, 180);
        }

        robot.intake.setOut();
        sleep(600);
        robot.intake.setOpen();
        robot.intake.setStop();


        // drive back
        if (iAmBlue()) {
            auto.encoderDrive(AutoHelper.DRIVE_SPEED, vuMark == RelicRecoveryVuMark.RIGHT? -8.0: -6.0, 3.0, true, 0);
        } else {
            auto.encoderDrive(AutoHelper.DRIVE_SPEED, vuMark == RelicRecoveryVuMark.LEFT? -8.0: -6.0, 3.0, true, 180);
        }

    }

    @Override
    public void driveToPile() throws InterruptedException {

        robot.lift.setLiftTop();
        auto.gyroTurn(AutoHelper.DRIVE_SPEED, iAmBlue()? 220 + angleAdjust : 320 + angleAdjust, AutoHelper.P_TURN_COEFF);

        auto.encoderDrive(AutoHelper.DRIVE_SPEED, 30 + distAdjust, 3.0, true, iAmBlue()? 220 + angleAdjust: 320 + angleAdjust);


        // Record drive motor encoder positions. Use these values to return to this position after collecting glyphs
        left1Pos = robot.leftDrive1.getCurrentPosition();
        left2Pos = robot.leftDrive2.getCurrentPosition();
        right1Pos = robot.rightDrive1.getCurrentPosition();
        right2Pos = robot.rightDrive2.getCurrentPosition();

        // Drive forward to collect glyph
        auto.collectGlyph(AutoHelper.DRIVE_SPEED, 18,3, true, iAmBlue()? 220 + angleAdjust : 320 + angleAdjust);
    }

    @Override
    public void loadFirstGlyph() throws InterruptedException {
        // Check if glyph is in intake
        if (robot.intake.distLeft() < 16.0 || robot.intake.distRight() < 16.0) {

            // square glyph
            auto.squareGlyph(1.0, -0.25,  2.0, 0.5);

            // intake on to hold glyph while driving back
            robot.intake.intakeLeftMotor.setPower(0.70);
            robot.intake.intakeRightMotor.setPower(0.70);

            auto.encoderDrive(AutoHelper.DRIVE_SPEED, -8.0, 3.0, true, iAmBlue()? 220 + angleAdjust : 320 + angleAdjust);

            // square glyph
            auto.squareGlyph(1.0, -0.25,  2.0, 0.5);

            robot.intake.setStop();

            // determine and record glyph color
            auto.firstGlyphColor = robot.intake.setGlyphColor();

            // Auto load glyph
            auto.autoLoadFirstGlyph(false);
            auto.glyphsCollected+=1;


        }
    }

    @Override
    public void attemptGlyphCollect() throws InterruptedException {
        auto.collectGlyph(AutoHelper.DRIVE_SPEED, 10, 3, true, iAmBlue()? 220 + angleAdjust : 320 + angleAdjust);
    }

    @Override
    public void collectSecondGlyph() throws InterruptedException {
        if (auto.autoTime.seconds() < 19) {
            auto.flipToLoadSecondGlyph();

            //robot.lift.setLiftHeight(1.0);

            auto.collectGlyph(AutoHelper.DRIVE_SPEED, 12,3, true, iAmBlue()? 220 + angleAdjust : 320 + angleAdjust);
        }
    }

    @Override
    public void loadSecondGlyph() throws InterruptedException {
        if (robot.intake.distLeft() < 16.0 || robot.intake.distRight() < 16.0) {

            // square glyph
            auto.squareGlyph(1.0, -0.25, 2.0, 0.5);

            // intake on to hold glyph while driving back
            robot.intake.intakeLeftMotor.setPower(0.70);
            robot.intake.intakeRightMotor.setPower(0.70);

            auto.encoderDrive(AutoHelper.DRIVE_SPEED, -8.0, 3.0, true, iAmBlue()? 220 + angleAdjust : 320 + angleAdjust);

            // square glyph
            auto.squareGlyph(1.0, -0.25, 2.0, 0.5);

            robot.intake.setStop();

            // determine and record glyph color
            auto.secondGlyphColor = robot.intake.setGlyphColor();

            // Auto load glyph
            auto.autoLoadSecondGlyph(vuMark);
            auto.glyphsCollected+=1;

        } else {
            // never detected glyph in intake. Back off and set intake out to clear any potential jams.
            robot.intake.setOut();
            auto.encoderDrive(AutoHelper.DRIVE_SPEED, -12.0, 2.0, true, iAmBlue()? 220 + angleAdjust : 320 + angleAdjust);
            robot.intake.setStop();
            robot.intake.setOpen();
            robot.lift.setLiftHeight(8.25);
            while(robot.lift.distFromBottom() < 7.5) idle();
            robot.gripper.flip();
            while(robot.gripper.isMoving()) sleep (1);
            robot.lift.setLiftHeight(1.0);
        }
    }
    @Override
    public void returnToBox() throws InterruptedException {
        int inches = auto.determineDistance(left1Pos, left2Pos, right1Pos, right2Pos);

        if (vuMark == RelicRecoveryVuMark.CENTER || vuMark == RelicRecoveryVuMark.UNKNOWN ) {
            auto.encoderDrive(AutoHelper.DRIVE_SPEED, -inches + -30, 5, true, iAmBlue()? 220 + angleAdjust : 320 + angleAdjust);
            robot.lift.setLiftHeight(0.0);
        }

        if (vuMark == RelicRecoveryVuMark.RIGHT) {
            auto.encoderDrive(AutoHelper.DRIVE_SPEED, -inches + -25 + -distAdjust, 5, true, iAmBlue()? 220 + angleAdjust : 320 + angleAdjust);
            robot.lift.setLiftHeight(0.0);
        }

        if (vuMark == RelicRecoveryVuMark.LEFT) {
            auto.encoderDrive(AutoHelper.DRIVE_SPEED, -inches + -25, 5, true, iAmBlue()? 220 + angleAdjust : 320 + angleAdjust);
            robot.lift.setLiftHeight(0.0);
        }

    }

    @Override
    public void placeExtraGlyphs() throws InterruptedException {

        if (auto.glyphsCollected > 0) {

            if (vuMark == RelicRecoveryVuMark.CENTER || vuMark == RelicRecoveryVuMark.UNKNOWN){
                // turn to place extra glyphs
                auto.gyroTurn(AutoHelper.DRIVE_SPEED, iAmBlue()? -31: 211, AutoHelper.P_TURN_COEFF_180);

                auto.autoTime.reset();
                // lift to floor
                while(!robot.lift.resetFloorPos() || auto.autoTime.milliseconds() < 150) sleep(1);
                robot.lift.liftMotor.setPower(0.0);

                // Extend gripper out
                robot.gripper.setExtendOut();
                sleep(200);

                auto.encoderDrive(AutoHelper.DRIVE_SPEED, 5.0, 3.0, true, iAmBlue()? -31: 211);

                // Drop glyphs
                robot.gripper.setBothOpen();
                robot.intake.setClosed();
                sleep(350);
            }
            if (vuMark == RelicRecoveryVuMark.RIGHT){
                // turn to place extra glyphs
                auto.gyroTurn(AutoHelper.DRIVE_SPEED, iAmBlue()? 41: 139, AutoHelper.P_TURN_COEFF_180);

                auto.autoTime.reset();
                // lift to floor
                while(!robot.lift.resetFloorPos() || auto.autoTime.milliseconds() < 150) sleep(1);
                robot.lift.liftMotor.setPower(0.0);

                // Extend gripper out
                robot.gripper.setExtendOut();
                sleep(200);

                auto.encoderDrive(AutoHelper.DRIVE_SPEED, 14.0, 3.0, true, iAmBlue()? 41: 139);

                // Drop glyphs
                robot.gripper.setBothOpen();
                robot.intake.setClosed();
                sleep(350);
            }
            if (vuMark == RelicRecoveryVuMark.LEFT){
                // turn to place extra glyphs
                auto.gyroTurn(AutoHelper.DRIVE_SPEED, iAmBlue()? -23: 203, AutoHelper.P_TURN_COEFF_180);

                auto.autoTime.reset();
                // lift to floor
                while(!robot.lift.resetFloorPos() || auto.autoTime.milliseconds() < 150) sleep(1);
                robot.lift.liftMotor.setPower(0.0);

                // Extend gripper out
                robot.gripper.setExtendOut();
                sleep(200);

                auto.encoderDrive(AutoHelper.DRIVE_SPEED, 9.0, 3.0, true, iAmBlue()? -23: 203);

                // Drop glyphs
                robot.gripper.setBothOpen();
                robot.intake.setClosed();
                sleep(350);
            }

        }

    }

    @Override
    public void park() throws InterruptedException {
        if (auto.glyphsCollected > 0) {

            if (vuMark == RelicRecoveryVuMark.CENTER || vuMark == RelicRecoveryVuMark.UNKNOWN){
                // Drive back to avoid being in contact with glyphs. Do not use gyro.
                auto.encoderDrive(AutoHelper.DRIVE_SPEED, -2.0, 3.0, false, 0.0);

            }

            if (vuMark == RelicRecoveryVuMark.RIGHT) {
                // Drive back to avoid being in contact with glyphs. Do not use gyro.
                auto.encoderDrive(AutoHelper.DRIVE_SPEED, -6.0, 3.0, false, 0.0);

            }

            if (vuMark == RelicRecoveryVuMark.LEFT) {
                // Drive back to avoid being in contact with glyphs. Do not use gyro.
                auto.encoderDrive(AutoHelper.DRIVE_SPEED, -6.0, 3.0, false, 0.0);

            }

        }
    }

    @Override
    public boolean iAmBlue() {
        return true;
    }
}
