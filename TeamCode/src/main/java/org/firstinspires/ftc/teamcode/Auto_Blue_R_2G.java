package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by jholly on 1/4/2018.
 *
 * Exactly the same as 2G version but DON"T try to intake any extra glyphs
 */

@Autonomous(name="Auto Blue R 2G", group="DM18")
@Disabled
public class Auto_Blue_R_2G extends Auto_Master {

    @Override
    public void driveToBox() throws InterruptedException {

        if (vuMark == RelicRecoveryVuMark.CENTER || vuMark == RelicRecoveryVuMark.UNKNOWN) {
            // Drive forward to lineup with center cryptoglyph
            if (iAmBlue()) {
                auto.encoderDrive(AutoHelper.DRIVE_SPEED_SLOW, 33.0, 5.0, true, 0.0);
            } else {
                auto.encoderDrive(AutoHelper.DRIVE_SPEED_SLOW, -36.0, 5.0, true, 0.0);
            }

        }
        if (vuMark == RelicRecoveryVuMark.RIGHT) {
            // Drive forward to lineup with center cryptoglyph
            if (iAmBlue()) {
                auto.encoderDrive(AutoHelper.DRIVE_SPEED_SLOW, 33.0 + 7.5, 5.0, true, 0.0);
            } else {
                auto.encoderDrive(AutoHelper.DRIVE_SPEED_SLOW, -36.0 + 7.5, 5.0, true, 0.0);
            }

        }
        if (vuMark == RelicRecoveryVuMark.LEFT) {
            // Drive forward to lineup with center cryptoglyph
            if (iAmBlue()) {
                auto.encoderDrive(AutoHelper.DRIVE_SPEED_SLOW, 33.0 - 7.5, 5.0, true, 0.0);
            } else {
                auto.encoderDrive(AutoHelper.DRIVE_SPEED_SLOW, -36.0 - 7.5, 5.0, true, 0.0);
            }

        }

        // Turn toward center cryptoglyph
        auto.gyroTurn(AutoHelper.TURN_SPEED, 90, AutoHelper.P_TURN_COEFF);
        // Outake glyph
        robot.gripper.setBothOpen();
        sleep(250);
        // Drive closer to center cryptoglyph
        auto.encoderDrive(AutoHelper.DRIVE_SPEED_SLOW, 7.0, 3.0, true, 90);

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
        auto.encoderDrive(AutoHelper.DRIVE_SPEED, -18.0, 3.0, true, 90);

        auto.gyroTurn(AutoHelper.TURN_SPEED,-90, AutoHelper.P_TURN_COEFF);

        // Record drive motor encoder positions. Use these values to return to this position after collecting glyphs
        left1Pos = robot.leftDrive1.getCurrentPosition();
        left2Pos = robot.leftDrive2.getCurrentPosition();
        right1Pos = robot.rightDrive1.getCurrentPosition();
        right2Pos = robot.rightDrive2.getCurrentPosition();

        // Drive forward to collect glyph
        auto.collectGlyph(0.3, 10,3, true, -90);

    }


    public void loadFirstGlyph() throws InterruptedException {
        // Check if glyph is in intake
        if (robot.intake.distLeft() < 13.0 || robot.intake.distRight() < 13.0) {

            // square glyph
            auto.squareGlyph(1.0, -0.25, 2.0, 0.5);

            // intake on to hold glyph while driving back
            robot.intake.intakeLeftMotor.setPower(0.30);
            robot.intake.intakeRightMotor.setPower(0.30);
            // drive backwards to avoid interference from other glyphs during load
            auto.encoderDrive(0.3, -3.0, 2.0, true, -90);

            robot.intake.setStop();

            // determine and record glyph color
            auto.firstGlyphColor = robot.intake.setGlyphColor();

            // Auto load glyph
            auto.autoLoadFirstGlyph(false);
            auto.glyphsCollected = 1;



        } else {
            // never detected glyph in intake. Back off and set intake out to clear any potential jams.
            robot.intake.setOut();
            auto.encoderDrive(0.3, -7.0, 3.0, true, -90);
            robot.intake.setStop();
        }
    }

    @Override
    public void returnToBox() throws InterruptedException {
        int inches = auto.determineDistance(left1Pos, left2Pos, right1Pos, right2Pos);

        auto.encoderDrive(AutoHelper.DRIVE_SPEED, -inches, 5, true, -90);

        // Turn toward cryptobox
        auto.gyroTurn(AutoHelper.TURN_SPEED, 90, AutoHelper.P_TURN_COEFF);

        auto.encoderDrive(AutoHelper.DRIVE_SPEED, 15, 5, true, 90);
    }

    @Override
    public void placeExtraGlyphs(){

        if (auto.glyphsCollected > 0){

            // Extend gripper out
            robot.gripper.setExtendOut();

            sleep(200);

            // Drop glyphs
            robot.gripper.setBothOpen();

            sleep (350);

            robot.gripper.setExtendIn();

            sleep (100);

            robot.lift.setLiftBtm();

        }
    }

    @Override
    public void park() throws InterruptedException {
        // Drive back, but stay in safe zone
        auto.encoderDrive(AutoHelper.DRIVE_SPEED, -4.0, 3.0, true, 90);

        robot.intake.setStop();
    }


}
