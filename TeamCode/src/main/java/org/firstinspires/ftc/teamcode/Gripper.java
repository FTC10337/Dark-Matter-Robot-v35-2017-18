package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 *    Everything related to the gripper
 */
public class Gripper {

    // Hardware
    public Servo purpleGrip = null;
    public Servo blackGrip = null;
    public Servo rotateServo = null;
    public Servo extendGrip = null;

    // Servo constants
    public final static double B_GRIP_OPEN = 0.72;
    public final static double B_GRIP_PARTIAL_OPEN = 0.80; // Changed to same as full close
    public final static double B_GRIP_CLOSED = 0.234;
    public final static double P_GRIP_OPEN = 0.80;
    public final static double P_GRIP_PARTIAL_OPEN = 0.80;
    public final static double P_GRIP_CLOSED = 0.234;

    public final static double GRIP_ROTATE_NORMAL = 0.745;
    public final static double GRIP_ROTATE_FLIPPED = .06;
    public final static double GRIP_EXTEND_HOME = 0.87;
    public final static double GRIP_EXTEND_OUT = 0.46;
    public final static double FLIP_TIME = 450;        // 1 second for servo to flip gripper
    public final static double GRIP_TIME = 225;        // 1 second for grip to open or close
    public final static double EXTEND_TIME = 250;

    /* Gripper state variables */
    Servo topGrip = purpleGrip;        // Should start w/ purple gripper on top
    Servo btmGrip = blackGrip;         // and black on bottom
    boolean isGripFlipped = false;

    /* Flip flipTimer */
    ElapsedTime flipTimer = new ElapsedTime();
    ElapsedTime purpleTimer = new ElapsedTime();
    ElapsedTime blackTimer = new ElapsedTime();
    ElapsedTime extendTimer = new ElapsedTime();
    ElapsedTime topTimer = null;
    ElapsedTime btmTimer = null;

    /**
     * Constructor
     */
    public void Gripper() {
        // Do nothing
    }

    /**
     * Initialize the gripper
     *
     * @param hw  Hardwaremap for our robot
     * @param pg  Name of purple gripper servo
     * @param bg  Name of black gripper servo
     * @param rot Name of gripper rotation servo
     */
    public void init(HardwareMap hw, String pg, String bg, String rot, String ext) {
        // Define and Initialize gripper servos
        purpleGrip = hw.servo.get(pg);
        blackGrip = hw.servo.get(bg);
        rotateServo = hw.servo.get(rot);
        extendGrip = hw.servo.get(ext);

        // Set the rotation servo for extended PWM range
        if (rotateServo.getController() instanceof ServoControllerEx) {
            // Confirm its an extended range servo controller before we try to set to avoid crash
            ServoControllerEx theControl = (ServoControllerEx) rotateServo.getController();
            int thePort = rotateServo.getPortNumber();
            PwmControl.PwmRange theRange = new PwmControl.PwmRange(553, 2500);
            theControl.setServoPwmRange(thePort, theRange);
        }

        // Set the black grip servo for extended PWM range
        if (blackGrip.getController() instanceof ServoControllerEx) {
            // Confirm its an extended range servo controller before we try to set to avoid crash
            ServoControllerEx theControl = (ServoControllerEx) blackGrip.getController();
            int thePort = blackGrip.getPortNumber();
            PwmControl.PwmRange theRange = new PwmControl.PwmRange(553, 2500);
            theControl.setServoPwmRange(thePort, theRange);
        }

        // Set the purple grip servo for extended PWM range
        if (purpleGrip.getController() instanceof ServoControllerEx) {
            // Confirm its an extended range servo controller before we try to set to avoid crash
            ServoControllerEx theControl = (ServoControllerEx) purpleGrip.getController();
            int thePort = rotateServo.getPortNumber();
            PwmControl.PwmRange theRange = new PwmControl.PwmRange(553, 2500);
            theControl.setServoPwmRange(thePort, theRange);
        }

        // Start with purple on top
        topGrip = purpleGrip;
        btmGrip = blackGrip;
        topTimer = purpleTimer;
        btmTimer = blackTimer;
        isGripFlipped = false;

        setFlipped(false);
        setExtendIn();
        setBothClosed();
    }

    /**
     * Open the purple gripper
     */
    public void setPurpleOpen() {

        purpleGrip.setPosition(P_GRIP_OPEN);
        purpleTimer.reset();
    }

    /**
     * Open the black gripper
     */
    public void setBlackOpen() {

        blackGrip.setPosition(B_GRIP_OPEN);
        blackTimer.reset();
    }

    /**
     * Open whichever gripper is currently on top
     */
    public void setTopOpen() {

        topGrip.setPosition(isGripFlipped ? B_GRIP_OPEN: P_GRIP_OPEN);
        topTimer.reset();
    }

    /**
     * Open whichever gripper is currently on bottom
     */
    public void setBtmOpen() {

        btmGrip.setPosition(isGripFlipped ? P_GRIP_OPEN: B_GRIP_OPEN);
        btmTimer.reset();
    }

    /**
     * Open both grippers
     */
    public void setBothOpen() {
        setTopOpen();
        setBtmOpen();
    }

    /**
     * Open both grippers partially to score without interferring with other stacked glyphs
     */
    public void setBothPartialOpen() {
        setBtmPartialOpen();
        setTopPartialOpen();
    }

    public void setBtmPartialOpen() {
        btmGrip.setPosition(isGripFlipped ? P_GRIP_PARTIAL_OPEN: B_GRIP_PARTIAL_OPEN);
        btmTimer.reset();
    }

    public void setTopPartialOpen() {
        topGrip.setPosition(isGripFlipped ? B_GRIP_PARTIAL_OPEN: P_GRIP_PARTIAL_OPEN);
        topTimer.reset();
    }

    /**
     * Extend gripper
     */
    public void setExtendOut() {

        extendGrip.setPosition(GRIP_EXTEND_OUT);
        extendTimer.reset();
    }

    public void moveInOut(double speed) {
        speed = Range.clip(speed, -1, 1);
        double current = extendGrip.getPosition();
        double target =  current + (((GRIP_EXTEND_OUT - GRIP_EXTEND_HOME) * speed) / 3);   // At full stick will take 20 cycles
        target = Range.clip(target, GRIP_EXTEND_OUT, GRIP_EXTEND_HOME);
        extendGrip.setPosition(target);
        extendTimer.reset();
    }

    /**
     * Retract gripper
     */
    public void setExtendIn() {

        extendGrip.setPosition(GRIP_EXTEND_HOME);
        extendTimer.reset();
    }

    /**
     * Close the purple gripper
     */
    public void setPurpleClosed() {

        purpleGrip.setPosition(P_GRIP_CLOSED);
        purpleTimer.reset();
    }

    /**
     * Close the black gripper
     */
    public void setBlackClosed() {

        blackGrip.setPosition(B_GRIP_CLOSED);
        blackTimer.reset();
    }

    /**
     * Close whichever gripper is currently on top
     */
    public void setTopClosed() {

        topGrip.setPosition(isGripFlipped ? B_GRIP_CLOSED: P_GRIP_CLOSED);
        topTimer.reset();
    }

    /**
     * Close whichever gripper is currently on bottom
     */
    public void setBtmClosed() {

        btmGrip.setPosition(isGripFlipped ? P_GRIP_CLOSED: B_GRIP_CLOSED);
        btmTimer.reset();
    }

    /**
     * Close both grippers
     */
    public void setBothClosed() {
        setTopClosed();
        setBtmClosed();
    }


    public boolean isTopClosed() {
        return (almostEqual(topGrip.getPosition(), (isGripFlipped ? B_GRIP_CLOSED: P_GRIP_CLOSED)));
    }

    public boolean isBtmClosed() {
        return (almostEqual(btmGrip.getPosition(), isGripFlipped ? P_GRIP_CLOSED: B_GRIP_CLOSED));
    }

    public boolean isPurpleClosed() {
        return (almostEqual(purpleGrip.getPosition(), B_GRIP_CLOSED));
    }

    public boolean isBlackClosed() {
        return (almostEqual(blackGrip.getPosition(), B_GRIP_CLOSED));
    }

    public boolean isBtmOpen() { return (almostEqual(btmGrip.getPosition(), isGripFlipped ? P_GRIP_OPEN: B_GRIP_OPEN)); }

    public boolean isTopOpen() { return (almostEqual(topGrip.getPosition(), isGripFlipped ? B_GRIP_OPEN: P_GRIP_OPEN)); }

    public boolean isBtmPartialOpen() { return (almostEqual(btmGrip.getPosition(), isGripFlipped ? P_GRIP_PARTIAL_OPEN: B_GRIP_PARTIAL_OPEN)); }

    public boolean isTopPartialOpen() { return (almostEqual(topGrip.getPosition(), isGripFlipped ? B_GRIP_PARTIAL_OPEN: P_GRIP_PARTIAL_OPEN)); }

    public boolean isPusherOut() { return (Math.abs(extendGrip.getPosition() - GRIP_EXTEND_HOME) > 0.02); }

    public void flip() {
        if (isGripFlipped) {
            // Was flipped so turn it back upright
            setFlipped(false);
        } else {
            // Was not flipped so turn it upside down
            setFlipped(true);
        }

    }

    public void setFlipped(boolean flipped) {
        if (flipped) {
            rotateServo.setPosition(GRIP_ROTATE_FLIPPED);
            isGripFlipped = true;
            topGrip = blackGrip;
            btmGrip = purpleGrip;
        }
        else {
            rotateServo.setPosition(GRIP_ROTATE_NORMAL);
            isGripFlipped = false;
            topGrip = purpleGrip;
            btmGrip = blackGrip;
        }
        flipTimer.reset();          // Start a flipTimer so we can check later if it might be moving
    }

    public boolean isFlipping() { return (flipTimer.milliseconds() < FLIP_TIME); }

    public boolean topIsMoving() {
        return (topTimer.milliseconds() < GRIP_TIME);
    }

    public boolean btmIsMoving() {
        return (btmTimer.milliseconds() < GRIP_TIME);
    }

    public boolean purpleIsMoving() {
        return (purpleTimer.milliseconds() < GRIP_TIME);
    }

    public boolean blackIsMoving() {
        return (blackTimer.milliseconds() < GRIP_TIME);
    }

    public boolean isMoving() {
        return (purpleIsMoving() || blackIsMoving() || btmIsMoving() || topIsMoving() || isFlipping());
    }

    public boolean isExtending() { return (extendTimer.milliseconds() < EXTEND_TIME);}

    /**
     * This is designed to be used for manual tuning of servo Min/Max constants not for routine use
     *
     * @return current top grip servo position
     */
    public double getTopServoPos() {
        return topGrip.getPosition();
    }

    /**
     * This is designed to be used for manual tuning of servo Min/Max constants not for routine use
     *
     * @return current btm grip servo position
     */

    public double getBtmServoPos() {
        return btmGrip.getPosition();
    }

    /**
     * This is designed to be used for manual tuning of servo Min/Max constants not for routine use
     *
     * @param pos desired top grip servo position
     */
    public void setTopServoPos(double pos) {
        pos = Range.clip(pos, 0.0, 1.0);
        topGrip.setPosition(pos);
    }

    /**
     * This is designed to be used for manual tuning of servo Min/Max constants not for routine use
     *
     * @param pos desired bottom grip servo position
     */
    public void setBtmServoPos(double pos) {
        pos = Range.clip(pos, 0.0, 1.0);
        btmGrip.setPosition(pos);
    }


    /**
     * This is designed to be used for manual tuning of servo Min/Max constants not for routine use
     *
     * @return current rotation servo position
     */
    public double getRotServoPos() {
        return rotateServo.getPosition();
    }

    /**
     * This is designed to be used for manual tuning of servo Min/Max constants not for routine use
     *
     * @param pos desired rotate servo position
     */
    public void setRotServoPos(double pos) {
        pos = Range.clip(pos, 0.0, 1.0);
        rotateServo.setPosition(pos);
    }


    public boolean almostEqual(double val1, double val2) {
        return (Math.abs(val1 - val2) < 0.02);
    }
}

