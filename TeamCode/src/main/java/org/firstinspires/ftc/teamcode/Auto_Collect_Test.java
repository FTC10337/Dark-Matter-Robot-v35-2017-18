package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by jholly on 1/4/2018.
 *
 * Exactly the same as 2G version but DON"T try to intake any extra glyphs
 */

@Autonomous(name="Auto Collect Test", group="DM18")
@Disabled
public class Auto_Collect_Test extends Auto_Master {

    @Override
    public void driveToBox() throws InterruptedException {
    }

    @Override
    public boolean iAmBlue() {
        return true;
    }

    @Override
    public void driveToPile() throws InterruptedException {

        robot.gripper.setBothOpen();
        // Record drive motor encoder positions. Use these values to return to this position after collecting glyphs
        left1Pos = robot.leftDrive1.getCurrentPosition();
        left2Pos = robot.leftDrive2.getCurrentPosition();
        right1Pos = robot.rightDrive1.getCurrentPosition();
        right2Pos = robot.rightDrive2.getCurrentPosition();

        // Drive forward to collect glyph
        auto.collectGlyph(AutoHelper.DRIVE_SPEED, 18,3, true, 0);

    }

    @Override
    public void loadFirstGlyph() throws InterruptedException {
        // Check if glyph is in intake
        if (robot.intake.distLeft() < 13.0 || robot.intake.distRight() < 13.0) {

            // square glyph
            auto.squareGlyph(1.0, -0.25,  2.0, 0.5);

            // intake on to hold glyph while driving back
            robot.intake.intakeLeftMotor.setPower(0.70);
            robot.intake.intakeRightMotor.setPower(0.70);

            auto.encoderDrive(AutoHelper.DRIVE_SPEED, -8.0, 3.0, true, 0);

            // square glyph
            auto.squareGlyph(1.0, -0.25,  2.0, 0.5);

            robot.intake.setStop();

            // determine and record glyph color
            auto.firstGlyphColor = robot.intake.setGlyphColor();

            // Auto load glyph
            auto.autoLoadFirstGlyph(false);
            auto.glyphsCollected+=1;


        } else {
            // never detected glyph in intake. Back off and set intake out to clear any potential jams.
            robot.intake.setOut();
            auto.encoderDrive(AutoHelper.DRIVE_SPEED, -7.0, 3.0, true, 0);
            robot.intake.setStop();
        }
    }

    @Override
    public void attemptGlyphCollect() throws InterruptedException {
            auto.collectGlyph(AutoHelper.DRIVE_SPEED, 10, 3, true, 0);
    }

    @Override
    public void collectSecondGlyph() throws InterruptedException {

        auto.flipToLoadSecondGlyph();

        //robot.lift.setLiftHeight(1.0);

        auto.collectGlyph(AutoHelper.DRIVE_SPEED, 12,3, true, 0);
    }

    @Override
    public void loadSecondGlyph() throws InterruptedException {
        if (robot.intake.distLeft() < 13.0 || robot.intake.distRight() < 13.0) {

            // square glyph
            auto.squareGlyph(1.0, -0.25, 2.0, 0.5);

            // intake on to hold glyph while driving back
            robot.intake.intakeLeftMotor.setPower(0.70);
            robot.intake.intakeRightMotor.setPower(0.70);

            auto.encoderDrive(AutoHelper.DRIVE_SPEED, -8.0, 3.0, true, 0);

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
            auto.encoderDrive(AutoHelper.DRIVE_SPEED, -8.0, 2.0, true, 0);
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

        auto.encoderDrive(AutoHelper.DRIVE_SPEED, -inches, 5, true, 0);

    }

    @Override
    public void placeExtraGlyphs() throws InterruptedException {

            robot.gripper.setBothOpen();
   }

    @Override
    public void park() throws InterruptedException {
          }

    @Override
    public void readyForRelic() {
        // Do nothing
    }
}
