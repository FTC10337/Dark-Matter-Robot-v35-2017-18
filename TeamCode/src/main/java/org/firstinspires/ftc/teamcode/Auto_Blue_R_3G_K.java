package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by jholly on 1/4/2018.
 *
 * Exactly the same as 2G version but DON"T try to intake any extra glyphs
 */

@Autonomous(name="Auto Blue R 3G_K", group="DM18")
//@Disabled
public class Auto_Blue_R_3G_K extends Auto_Master {

    @Override
    public void driveToBox() throws InterruptedException {

        if (vuMark == RelicRecoveryVuMark.CENTER || vuMark == RelicRecoveryVuMark.UNKNOWN) {
            // Drive forward to lineup with center cryptoglyph
            if (iAmBlue()) {
                auto.encoderDrive(0.5, 33.0, 5.0, true, 0.0);
            } else {
                auto.encoderDrive(0.5, -36.0, 5.0, true, 0.0);
            }

        }
        if (vuMark == RelicRecoveryVuMark.RIGHT) {
            // Drive forward to lineup with center cryptoglyph
            if (iAmBlue()) {
                auto.encoderDrive(0.5, 33.0 + 7.5, 5.0, true, 0.0);
            } else {
                auto.encoderDrive(0.5, -36.0 + 7.5, 5.0, true, 0.0);
            }

        }
        if (vuMark == RelicRecoveryVuMark.LEFT) {
            // Drive forward to lineup with center cryptoglyph
            if (iAmBlue()) {
                auto.encoderDrive(0.5, 33.0 - 7.5, 5.0, true, 0.0);
            } else {
                auto.encoderDrive(0.5, -36.0 - 7.5, 5.0, true, 0.0);
            }

        }

        // Turn toward center cryptoglyph
        auto.gyroTurn(0.8, 90, auto.P_TURN_COEFF);
        // Outake glyph
        robot.gripper.setBothOpen();
        sleep(250);
        // Drive closer to center cryptoglyph
        auto.encoderDrive(0.5, 7.0, 3.0, true, 90);

        robot.intake.setOut();
        sleep(500);
        robot.intake.setOpen();
        robot.intake.setStop();
    }


    @Override
    public boolean iAmBlue() {
        return true;
    }

    @Override
    public void driveToPile() throws InterruptedException {
        auto.encoderDrive(0.6, -18.0, 3.0, true, 90);

        auto.gyroTurn(0.8,-90, auto.P_TURN_COEFF);

        // Record drive motor encoder positions. Use these values to return to this position after collecting glyphs
        left1Pos = robot.leftDrive1.getCurrentPosition();
        left2Pos = robot.leftDrive2.getCurrentPosition();
        right1Pos = robot.rightDrive1.getCurrentPosition();
        right2Pos = robot.rightDrive2.getCurrentPosition();

        // Drive forward to collect glyph
        auto.collectGlyph(0.3, 10,3, true, -90);

    }

    @Override
    public void loadFirstGlyph() throws InterruptedException {
        // Check if glyph is in intake
        if (robot.intake.distLeft() < 12.0 || robot.intake.distRight() < 12.0) {

            // square glyph
            auto.squareGlyph(1.0, -0.15, 2);

            // intake on to hold glyph while driving back
            robot.intake.intakeLeftMotor.setPower(0.30);
            robot.intake.intakeRightMotor.setPower(0.30);

            // drive backwards to avoid interference from other glyphs during load
            auto.encoderDrive(0.3, -3.0, 2.0, true, -90);

            robot.intake.setStop();

            // Auto load glyph
            auto.autoLoadFirstGlyph();
            auto.glyphsCollected = 1;

            // determine and record glyph color
            auto.firstGlyphColor = robot.intake.setGlyphColor();

        } else {
            // never detected glyph in intake. Back off and set intake out to clear any potential jams.
            robot.intake.setOut();
            auto.encoderDrive(0.3, -7.0, 3.0, true, -90);
            robot.intake.setStop();
        }
    }

    @Override
    public void collectSecondGlyph() throws InterruptedException {

        auto.flpToLoadSecondGlyph();

        robot.lift.setLiftHeight(1.0);

        auto.collectGlyph(0.3, 6,3, true, -90);
    }

    @Override
    public void loadSecondGlyph() throws InterruptedException {
        if (robot.intake.distLeft() < 10.0 || robot.intake.distRight() < 10.0) {

            // square glyph
            auto.squareGlyph(1.0, -0.15, 2);

            // intake on to hold glyph while driving back
            robot.intake.intakeLeftMotor.setPower(0.30);
            robot.intake.intakeRightMotor.setPower(0.30);
            // drive backwards to avoid interference from other glyphs during load
            auto.encoderDrive(0.3, -3.0, 2.0, true, -90);

            robot.intake.setStop();

            // determine and record glyph color
            auto.secondGlyphColor = robot.intake.setGlyphColor();

            // Auto load glyph
            auto.autoLoadSecondGlyph();
            auto.glyphsCollected = 2;

        } else {
            // never detected glyph in intake. Back off and set intake out to clear any potential jams.
            robot.intake.setOut();
            robot.intake.setStop();
            robot.lift.setLiftHeight(8.0);
            while(robot.lift.distFromBottom() < 7.5) idle();
            robot.gripper.flip();
            while(robot.gripper.isMoving()) idle();
            robot.lift.setLiftBtm();
        }
    }

    @Override
    public void returnToBox() throws InterruptedException {
        int inches = auto.determineDistance(left1Pos, left2Pos, right1Pos, right2Pos);

        auto.encoderDrive(0.8, -inches, 5, true, -90);

        // Turn toward cryptobox
        auto.gyroTurn(0.8, 90, auto.P_TURN_COEFF);

        auto.encoderDrive(0.8, 15, 5, true, 90);
    }

    @Override
    public void park() throws InterruptedException {

        robot.lift.setLiftBtm();
    }


}
