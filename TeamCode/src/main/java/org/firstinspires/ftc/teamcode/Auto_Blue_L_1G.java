package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by jholly on 1/4/2018.
 *
 * Exactly the same as 2G version but DON"T try to intake any extra glyphs
 */

@Autonomous(name="Auto Blue L 1G", group="DM18")
@Disabled
public class Auto_Blue_L_1G extends Auto_Master {
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
            auto.encoderDrive(AutoHelper.DRIVE_SPEED, iAmBlue()? 13.515 : 12.165, 5.0, true, -90.0);

        }
        if (vuMark == RelicRecoveryVuMark.RIGHT) {

            angleAdjust = -10;
            distAdjust= iAmBlue()? -3: 5;

            // Drive forward to lineup with center cryptoglyph
            auto.encoderDrive(AutoHelper.DRIVE_SPEED, iAmBlue()? 13.515 + 7.5 : 12.165 - 7.5, 5.0, true, -90.0);

        }
        if (vuMark == RelicRecoveryVuMark.LEFT) {

            angleAdjust = 9;
            distAdjust = iAmBlue()? 5: -3;

            // Drive forward to lineup with center cryptoglyph

            auto.encoderDrive(AutoHelper.DRIVE_SPEED, iAmBlue()? 13.515 - 7.5 : 12.165 + 7.5, 5.0, true, -90.0);

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
    // Do nothing
    }

    @Override
    public void loadFirstGlyph() throws InterruptedException {
        // Do nothing
    }

    @Override
    public void attemptGlyphCollect() throws InterruptedException {
        // Do nothing
    }

    @Override
    public void collectSecondGlyph() throws InterruptedException {
        // Do nothing
    }

    @Override
    public void loadSecondGlyph() throws InterruptedException {
        // Do nothing
    }
    @Override
    public void returnToBox() throws InterruptedException {
        // Do nothing

    }

    @Override
    public void placeExtraGlyphs() throws InterruptedException {

        // Do nothing

    }

    @Override
    public void park() throws InterruptedException {
        // Do nothing
    }

    @Override
    public boolean iAmBlue() {
        return true;
    }
}
