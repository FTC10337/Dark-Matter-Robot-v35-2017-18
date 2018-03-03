package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by jholly on 1/4/2018.
 *
 * Exactly the same as 2G version but DON"T try to intake any extra glyphs
 */

@Autonomous(name="Auto Blue R 3G Ground", group="DM18")
@Disabled
public class Auto_Blue_R_3G_Ground extends Auto_Master {

    @Override
    public void driveToBox() throws InterruptedException {

        if (vuMark == RelicRecoveryVuMark.CENTER || vuMark == RelicRecoveryVuMark.UNKNOWN) {

            angleAdjust = -25;
            // Drive forward to lineup with center cryptoglyph
            if (iAmBlue()) {
                auto.encoderDrive(AutoHelper.DRIVE_SPEED_SLOW, 33.0, 5.0, true, 0.0);
            } else {
                auto.encoderDrive(AutoHelper.DRIVE_SPEED_SLOW, -35.0, 5.0, true, 0.0);
            }

        }
        if (vuMark == RelicRecoveryVuMark.RIGHT) {

            angleAdjust = 30;
            // Drive forward to lineup with center cryptoglyph
            if (iAmBlue()) {
                auto.encoderDrive(AutoHelper.DRIVE_SPEED_SLOW, 33.0 + 7.5, 5.0, true, 0.0);
            } else {
                auto.encoderDrive(AutoHelper.DRIVE_SPEED_SLOW, -35.0 + 7.5, 5.0, true, 0.0);
            }

        }
        if (vuMark == RelicRecoveryVuMark.LEFT) {

            angleAdjust = -30;
            // Drive forward to lineup with center cryptoglyph
            if (iAmBlue()) {
                auto.encoderDrive(AutoHelper.DRIVE_SPEED_SLOW, 33.0 - 7.5, 5.0, true, 0.0);
            } else {
                auto.encoderDrive(AutoHelper.DRIVE_SPEED_SLOW, -35.0 - 7.5, 5.0, true, 0.0);
            }

        }

        // Turn toward center cryptoglyph
        auto.gyroTurn(AutoHelper.TURN_SPEED, 90, AutoHelper.P_TURN_COEFF);
        // Outake glyph
        robot.gripper.setBothOpen();
        sleep(250);
        // Drive closer to center cryptoglyph
        auto.encoderDrive(AutoHelper.DRIVE_SPEED, 7.0, 3.0, true, 90);

        robot.intake.setOut();
        sleep(600);
        robot.intake.setOpen();
        robot.intake.setStop();
    }


    @Override
    public boolean iAmBlue() {
        return true;
    }

    @Override
    public void driveToPile() throws InterruptedException {
        auto.encoderDrive(AutoHelper.DRIVE_SPEED, -18.0, 3.0, true, 90);

        auto.gyroTurn(AutoHelper.TURN_SPEED,-90, AutoHelper.P_TURN_COEFF);

        // Record drive motor encoder positions. Use these values to return to this position after collecting glyphs
        left1Pos = robot.leftDrive1.getCurrentPosition();
        left2Pos = robot.leftDrive2.getCurrentPosition();
        right1Pos = robot.rightDrive1.getCurrentPosition();
        right2Pos = robot.rightDrive2.getCurrentPosition();

        // Drive forward to collect glyph
        auto.collectGlyph(AutoHelper.DRIVE_SPEED, 18,3, true, -90);

    }

    @Override
    public void loadFirstGlyph() throws InterruptedException {
        // Check if glyph is in intake
        if (robot.intake.distLeft() < 13.0 || robot.intake.distRight() < 13.0) {

            // square glyph
            auto.squareGlyph(1.0, -0.25, 2.0,.5);

            // intake on to hold glyph while driving back
            robot.intake.intakeLeftMotor.setPower(0.70);
            robot.intake.intakeRightMotor.setPower(0.70);

            auto.encoderDrive(AutoHelper.DRIVE_SPEED, -4.0, 3.0, true, -90);

            // square glyph
            auto.squareGlyph(1.0, -0.25, 2.0,1.0);

            robot.intake.setStop();

            // Auto load glyph
            auto.autoLoadFirstGlyph(false);
            auto.glyphsCollected = 1;

            // determine and record glyph color
            auto.firstGlyphColor = robot.intake.setGlyphColor();

        } else {
            // never detected glyph in intake. Back off and set intake out to clear any potential jams.
            robot.intake.setOut();
            auto.encoderDrive(AutoHelper.DRIVE_SPEED, -7.0, 3.0, true, -90);
            robot.intake.setStop();
        }
    }

    @Override
    public void attemptGlyphCollect() throws InterruptedException {
            auto.collectGlyph(AutoHelper.DRIVE_SPEED, 10, 3, true, -90);
    }

    @Override
    public void collectSecondGlyph() throws InterruptedException {

        auto.flipToLoadSecondGlyph();

        //robot.lift.setLiftHeight(1.0);

        auto.collectGlyph(AutoHelper.DRIVE_SPEED, 12,3, true, -90);
    }

    @Override
    public void loadSecondGlyph() throws InterruptedException {
        if (robot.intake.distLeft() < 13.0 || robot.intake.distRight() < 13.0) {

            // square glyph
            auto.squareGlyph(1.0, -0.25, 2.0, 0.5);

            // intake on to hold glyph while driving back
            robot.intake.intakeLeftMotor.setPower(0.70);
            robot.intake.intakeRightMotor.setPower(0.70);

            auto.encoderDrive(AutoHelper.DRIVE_SPEED, -8.0, 3.0, true, -90);

            // square glyph
            auto.squareGlyph(1.0, -0.25, 2.0,0.5);

            robot.intake.setStop();

            // Auto load glyph
            auto.autoLoadSecondGlyph(vuMark);
            auto.glyphsCollected = 2;
            // determine and record glyph color
            auto.secondGlyphColor = robot.intake.setGlyphColor();

        } else {
            // never detected glyph in intake. Back off and set intake out to clear any potential jams.
            robot.intake.setOut();
            auto.encoderDrive(AutoHelper.DRIVE_SPEED, -7.0, 2.0, true, -90);
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

        auto.encoderDrive(AutoHelper.DRIVE_SPEED, -inches, 5, true, -90);

        // Turn toward cryptobox
        auto.gyroTurn(AutoHelper.TURN_SPEED, 90, AutoHelper.P_TURN_COEFF);

        if (vuMark == RelicRecoveryVuMark.CENTER || vuMark == RelicRecoveryVuMark.UNKNOWN) {
            auto.encoderDrive(AutoHelper.DRIVE_SPEED, 9.5, 5, true, 90);
        }
    }

    @Override
    public void placeExtraGlyphs() throws InterruptedException {

        // Check to see if there is enough time in auto to push glyphs in. This prevents being in contact with previous scored glyph at end of auto.
        if (auto.glyphsCollected > 0 && auto.autoTime.seconds() < 28){

            robot.lift.setLiftHeight(1.0);

            // Turn to place glyph on ground
            auto.gyroTurn(1.0, 90 + angleAdjust, AutoHelper.P_TURN_COEFF);

            // Turn and drive forward to ensure glyphs have been pushed into crytobox
            if (vuMark == RelicRecoveryVuMark.CENTER || vuMark == RelicRecoveryVuMark.UNKNOWN)
            {
                auto.encoderDrive(AutoHelper.DRIVE_SPEED, 4, 2, true, 90 + angleAdjust);
            }
            if (vuMark == RelicRecoveryVuMark.LEFT || vuMark == RelicRecoveryVuMark.RIGHT){
                auto.encoderDrive(AutoHelper.DRIVE_SPEED, 17, 2, true, 90 + angleAdjust);
            }
            // Extend gripper out
            robot.gripper.setExtendOut();

            sleep(200);

            // Drop glyphs
            robot.gripper.setBothOpen();

            sleep(350);

            if (vuMark == RelicRecoveryVuMark.CENTER || vuMark == RelicRecoveryVuMark.UNKNOWN)
            {
                auto.encoderDrive(AutoHelper.DRIVE_SPEED, 3.0, 1.5, true, 90 + angleAdjust);
                auto.encoderDrive(AutoHelper.DRIVE_SPEED, -5, 2, false, 90 + angleAdjust);
            }
            if (vuMark == RelicRecoveryVuMark.LEFT || vuMark == RelicRecoveryVuMark.RIGHT){
                auto.encoderDrive(AutoHelper.DRIVE_SPEED, 3.0, 1.5, true, 90 + angleAdjust);
                auto.encoderDrive(AutoHelper.DRIVE_SPEED, -5, 2, false, 90 + angleAdjust);
            }

        } else if (auto.glyphsCollected == 0 && (vuMark == RelicRecoveryVuMark.LEFT || vuMark == RelicRecoveryVuMark.RIGHT)) {
            auto.encoderDrive(AutoHelper.DRIVE_SPEED, 11.0, 5, true, 90);
        }
   }

    @Override
    public void park() throws InterruptedException {
        if (auto.glyphsCollected == 0){
            auto.gyroTurn(1.0, -90, AutoHelper.P_TURN_COEFF);
        }
          }

    @Override
    public void readyForRelic() {
        // Do nothing
    }
}
