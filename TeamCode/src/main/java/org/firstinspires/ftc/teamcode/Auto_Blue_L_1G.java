package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by jholly on 1/4/2018.
 *
 * Exactly the same as 2G version but DON"T try to intake any extra glyphs
 */

@Autonomous(name="Auto Blue L 1G", group="DM18")
//@Disabled
public class Auto_Blue_L_1G extends Auto_Master {
    @Override
    public void driveToBox() throws InterruptedException {

        // Drive off stone
        if (iAmBlue()) {
            auto.encoderDrive(0.5, 25.0, 5.0, true, 0.0);
            auto.gyroTurn(0.8, -90, auto.P_TURN_COEFF);
        } else {
            auto.encoderDrive(0.5, -25.0, 5.0, true, 0.0);
            auto.gyroTurn(0.8, -90, auto.P_TURN_COEFF);

        }


        if (vuMark == RelicRecoveryVuMark.CENTER || vuMark == RelicRecoveryVuMark.UNKNOWN) {
            // Drive forward to lineup with center cryptoglyph
            if (iAmBlue()) {
                auto.encoderDrive(0.5, 11.0, 5.0, true, -90.0);
            } else {
                auto.encoderDrive(0.5, 11.0, 5.0, true, -90.0);
            }

        }
        if (vuMark == RelicRecoveryVuMark.RIGHT) {
            // Drive forward to lineup with center cryptoglyph
            if (iAmBlue()) {
                auto.encoderDrive(0.5, 11.0 + 7.5, 5.0, true, -90.0);
            } else {
                auto.encoderDrive(0.5, 11.0 + 7.5, 5.0, true, -90.0);
            }

        }
        if (vuMark == RelicRecoveryVuMark.LEFT) {
            // Drive forward to lineup with center cryptoglyph
            if (iAmBlue()) {
                auto.encoderDrive(0.5, 11.0 - 7.5, 5.0, true, -90.0);
            } else {
                auto.encoderDrive(0.5, 11.0 - 7.5, 5.0, true, -90.0);
            }

        }

        // Turn toward  cryptoglyph
        if (iAmBlue()) {
            auto.gyroTurn(0.8, 0, auto.P_TURN_COEFF);
        } else {
            auto.gyroTurn(0.8, 180, auto.P_TURN_COEFF);
        }
        // Outake glyph
        robot.gripper.setBothOpen();
        sleep(250);
        // Drive closer to  cryptoglyph
        if (iAmBlue()) {
            auto.encoderDrive(0.5, 7.0, 3.0, true, 0);
        } else {
            auto.encoderDrive(0.5, 7.0, 3.0, true, 180);
        }

        robot.intake.setOut();
        sleep(500);
        robot.intake.setOpen();
        robot.intake.setStop();
    }

    @Override
    public void park() throws InterruptedException {
        // Drive back, but stay in safe zone
        if (iAmBlue()) {
            auto.encoderDrive(0.6, -8.0, 3.0, true, 0);
        } else {
            auto.encoderDrive(0.6, -8.0, 3.0, true, 180);

        }

        robot.intake.setStop();
    }

    @Override
    public boolean iAmBlue() {
        return true;
    }
}
