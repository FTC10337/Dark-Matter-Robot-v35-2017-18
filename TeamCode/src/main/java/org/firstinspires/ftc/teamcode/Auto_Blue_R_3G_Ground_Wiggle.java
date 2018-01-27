package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by mike on 1/26/2018.
 */


@Autonomous(name="Auto Blue R 3G Wiggle", group="DM18")
@Disabled


public class Auto_Blue_R_3G_Ground_Wiggle extends Auto_Blue_R_3G_Ground {

    final double  WIGGLE_ADJ = 5.0;         // How much off course to start wiggle driving

    @Override
    public void placeExtraGlyphs() throws InterruptedException {

        double wiggleAngle = 0.0;

        // Check to see if there is enough time in auto to push glyphs in. This prevents being in contact with previous scored glyph at end of auto.
        if (auto.glyphsCollected > 0 && auto.autoTime.seconds() < 28){

            robot.lift.setLiftHeight(1.0);

            // Oversteer to start in the direction we are already heading
            wiggleAngle = (angleAdjust>0)? WIGGLE_ADJ : -WIGGLE_ADJ;

            // Turn to place glyph on ground
            auto.gyroTurn(1.0, 90 + angleAdjust + wiggleAngle, AutoHelper.P_TURN_COEFF);

            // Turn and drive forward to ensure glyphs have been pushed into crytobox
            if (vuMark == RelicRecoveryVuMark.CENTER || vuMark == RelicRecoveryVuMark.UNKNOWN)
            {
                // Drive using aggressive distance correction to make path wiggle
                auto.encoderDrive(AutoHelper.DRIVE_SPEED, 7, 2, true, 90 + angleAdjust, true);
            }
            if (vuMark == RelicRecoveryVuMark.LEFT || vuMark == RelicRecoveryVuMark.RIGHT){
                // Drive using aggressive distance correction to make path wiggle
                auto.encoderDrive(AutoHelper.DRIVE_SPEED, 20, 2, true, 90 + angleAdjust, true);
            }
            // Extend gripper out
            robot.gripper.setExtendOut();

            sleep(200);

            // Drop glyphs
            robot.gripper.setBothOpen();

            sleep(350);

            if (vuMark == RelicRecoveryVuMark.CENTER || vuMark == RelicRecoveryVuMark.UNKNOWN)
            {
                auto.encoderDrive(AutoHelper.DRIVE_SPEED, -5, 2, false, 90 + angleAdjust);
            }
            if (vuMark == RelicRecoveryVuMark.LEFT || vuMark == RelicRecoveryVuMark.RIGHT){
                auto.encoderDrive(AutoHelper.DRIVE_SPEED, -5, 2, false, 90 + angleAdjust);
            }

        } else if (auto.glyphsCollected == 0 && (vuMark == RelicRecoveryVuMark.LEFT || vuMark == RelicRecoveryVuMark.RIGHT)) {
            auto.encoderDrive(AutoHelper.DRIVE_SPEED, 11.0, 5, true, 90);
        }



    }
}
