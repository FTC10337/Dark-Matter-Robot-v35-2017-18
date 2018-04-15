package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by jholly on 1/4/2018.
 *
 * Exactly the same as 2G version but DON"T try to intake any extra glyphs
 */

@Autonomous(name="Auto Turn Test", group="DM18")
@Disabled
public class Auto_Turn_Test extends Auto_Master {

    @Override
    public void driveToBox() throws InterruptedException {

/*
        auto.gyroTurn(1.0, 90, AutoHelper.P_TURN_COEFF);

        sleep (1000);

        if (Math.abs(90 - auto.readGyro()) > 1.0) {
            auto.gyroTurn(1.0, 90, AutoHelper.P_TURN_COEFF_STRONG);
        }

        sleep(2000);

        auto.gyroTurn(1.0, -90, AutoHelper.P_TURN_COEFF_180);

        sleep (1000);

        if (Math.abs(-90 - auto.readGyro()) > 1.0) {
            auto.gyroTurn(1.0, -90, AutoHelper.P_TURN_COEFF_STRONG);
        }

        sleep(2000);

        auto.gyroTurn(1.0, 0, AutoHelper.P_TURN_COEFF);

        sleep (1000);

        if (Math.abs(0 - auto.readGyro()) > 1.0) {
            auto.gyroTurn(1.0, 0, AutoHelper.P_TURN_COEFF_STRONG);
        }

        sleep(2000);

        auto.gyroTurn(1.0, 1, AutoHelper.P_TURN_COEFF_STRONG);

        sleep (500);

        auto.gyroTurn(1.0, 2, AutoHelper.P_TURN_COEFF_STRONG);

        sleep (500);

        auto.gyroTurn(1.0, 3, AutoHelper.P_TURN_COEFF_STRONG);

        sleep (500);

        auto.gyroTurn(1.0, 4, AutoHelper.P_TURN_COEFF_STRONG);

        sleep (500);

        auto.gyroTurn(1.0, 5, AutoHelper.P_TURN_COEFF_STRONG);

*/
    }


    @Override
    public boolean iAmBlue() {
        return true;
    }

    @Override
    public void driveToPile() throws InterruptedException {


        auto.encoderDrive(1.0, 72.0, 10.0, true, 0.0);

        sleep(1000);

        auto.encoderDrive(1.0, -72.0, 10.0, true, 0.0);

        sleep(1000);

        auto.encoderDrive(1.0, 72.0, 10.0, true, 0.0);

        sleep(1000);

        auto.encoderDrive(1.0, -72.0, 10.0, true, 0.0);

        sleep(1000);
    }

    @Override
    public void loadFirstGlyph() throws InterruptedException {

    }

    @Override
    public void attemptGlyphCollect() throws InterruptedException {

    }

    @Override
    public void collectSecondGlyph() throws InterruptedException {
    }

    @Override
    public void loadSecondGlyph() throws InterruptedException {

    }

    @Override
    public void returnToBox() throws InterruptedException {

    }

    @Override
    public void placeExtraGlyphs() throws InterruptedException {

    }

    @Override
    public void park() throws InterruptedException {

    }

    @Override
    public void readyForRelic() {
        // Do nothing
    }
}