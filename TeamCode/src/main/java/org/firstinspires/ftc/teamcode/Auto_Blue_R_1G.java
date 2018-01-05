package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by jholly on 1/4/2018.
 *
 * Exactly the same as 2G version but DON"T try to intake any extra glyphs
 */

@Autonomous(name="Auto Blue R 1G", group="DM18")
//@Disabled
public class Auto_Blue_R_1G extends Auto_Master {

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
    }


    @Override
    public boolean iAmBlue() {
        return true;
    }

    @Override
    public void park() throws InterruptedException {
        // Drive back, but stay in safe zone
        auto.encoderDrive(0.6, -8.0, 3.0, true, 90);

        robot.intake.setStop();
    }
}
