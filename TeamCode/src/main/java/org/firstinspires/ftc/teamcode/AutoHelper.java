package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by jholly on 12/5/2017.
 */

public class AutoHelper {

    /* Declare OpMode members. */
    HardwareDM18         robot;   // Use a Pushbot's hardware
    HardwareMap          hardwareMap;
    LinearOpMode         opMode;
    boolean              weAreBlue;

    private ElapsedTime     runtime = new ElapsedTime();

    public static final String TAG = "Vuforia VuMark Sample";

    OpenGLMatrix lastLocation = null;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;
    VuforiaTrackable relicTemplate;
    VuforiaTrackables relicTrackables;




    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.8;     // Nominal speed for auto moves.
    static final double     DRIVE_SPEED_SLOW        = 0.65;     // Slower speed where required
    static final double     TURN_SPEED              = 0.8;     // Turn speed

    static final double     HEADING_THRESHOLD       = 2 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.011;   // Larger is more responsive, but also less accurate
    static final double     P_DRIVE_COEFF_1         = 0.01;  // Larger is more responsive, but also less accurate
    static final double     P_DRIVE_COEFF_2         = 0.01;

    // Variables used for reading Gyro
    Orientation angles;
    double                  headingBias = 0.0;            // Gyro heading adjustment

    // Color Sensor Values
    static final double     BLUE_MIN                = -221.0;
    static final double     BLUE_MAX                = -139.0;
    static final double     RED_MIN                 = -21.0;
    static final double     RED_MAX                 = 21.0;

    // Variable for number of glyphs collected

    int glyphsCollected = 0;

    // Timer for detecting jewel color
    ElapsedTime             detectColorTimer        = new ElapsedTime();

    // Jewel color detection variable
    boolean                 detectedJewelColor      = false;

    // Storage for reading adaFruit color sensor for beacon sensing
    // adaHSV is an array that will hold the hue, saturation, and value information.
    float[] adaHSV = {0F, 0F, 0F};
    // adaValues is a reference to the adaHSV array.
    final float adaValues[] = adaHSV;


    /**
     *   Constructor
     */
    public AutoHelper() {
    }


    public void init(HardwareDM18 r, HardwareMap hw, LinearOpMode op, boolean blue) {
        // Remember the robot and hardareMap references
        robot = r;
        hardwareMap = hw;
        opMode = op;
        weAreBlue = blue;

        // Force reset the drive train encoders.  Do it twice as sometimes this gets missed due to USB congestion
        robot.setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        RobotLog.i("DM10337 -- Drive train encoders reset");

        RobotLog.i("DM10337- Finished Init");



        /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        /*
         * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
         * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
         * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
         * web site at https://developer.vuforia.com/license-manager.
         *
         * Vuforia license keys are always 380 characters long, and look as if they contain mostly
         * random data. As an example, here is a example of a fragment of a valid key:
         *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
         * Once you've obtained a license key, copy the string from the Vuforia web site
         * and paste it in to your code onthe next line, between the double quotes.
         */
        parameters.vuforiaLicenseKey = "AUt1Gf7/////AAAAGYGzxJPM7kkwgvjUoJXLxlZEJ9MMmT+9SHOR4KceMDfzyFZGLeLoQnB63pcWT0XT87KgeSQCKD7cV0LtdqEkgGGC3TFqW5W5d/VBctIrBtRd8hdIydaIJslqHa9G7itg1/lr94VhCA8niUgqpBB8dPx8PQ6zrUHBdJrQnczLvmxOSrbDzM6vCgmqyK1sfEsmN+GBux+0as/1a1Y6TIhHQDKZ7mCi8+nBB2mZYjtVVD3sDpPcEulol15QUhLli4PVFY1JB64jc2YqHX7IigSXbFQW1+hRMusVN5ZQjTxb1/OLqQsLQIApblHRYKO6896HYQwdHTQET6H5IFFEP6vgeFnfNk3Mv3VgabjOeT3IaDzG";

        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
    }

    public void processStartButton() {
        relicTrackables.activate();
        RobotLog.i("DM10337- Auto Pressed Start");
        // Step through each leg of the path,

        // Make sure the gyro is zeroed
        zeroGyro();

        RobotLog.i("DM10337 - Gyro bias set to " + headingBias);

        robot.lift.resetFloorPos();

    }


    public void processJewel() {
        double armPos = robot.jewelServo.getPosition();
        double armIncr = (robot.JEWEL_DEPLOY - armPos)/50;
        while (armPos < robot.JEWEL_DEPLOY) {
            armPos += armIncr;
            robot.jewelServo.setPosition(armPos);
            sleep(20);
        }

        robot.jewelCS.enableLed(true);
        sleep(500);

        // Check jewel color
        int jewelColor = jewelHue();

        // Check if we see blue or red
        if (jewelColor == -1) {
            // We see red
            robot.jewelRotServo.setPosition(iAmBlue()?robot.JEWEL_ROT_REV:robot.JEWEL_ROT_FWD);
            detectedJewelColor = true;
        } else if (jewelColor == 1) {
            // We see blue
            robot.jewelRotServo.setPosition(iAmBlue()?robot.JEWEL_ROT_FWD:robot.JEWEL_ROT_REV);
            detectedJewelColor = true;
        }

        sleep(500);

        // Reset jewel arm
        robot.jewelRotServo.setPosition(robot.JEWEL_ROT_HOME);
        armPos = robot.jewelServo.getPosition();
        armIncr = (robot.JEWEL_HOME - armPos)/50;
        while (armPos > robot.JEWEL_HOME) {
            armPos += armIncr;
            robot.jewelServo.setPosition(armPos);
            sleep(20);
        }

        robot.jewelCS.enableLed(false);

    }

    /**
     * Abbreviated call to encoderDrive w/o range aggressive turning or finding adjustments
     *
     * @param speed
     * @param distance
     * @param timeout
     * @param useGyro
     * @param heading
     * @throws InterruptedException
     */
    public void encoderDrive(double speed,
                             double distance,
                             double timeout,
                             boolean useGyro,
                             double heading) throws InterruptedException {
        encoderDrive(speed, distance, timeout, useGyro, heading, false, false, 0.0);
    }


    /**
     * Abbreviated call to encoderDrive w/o range finding adjustments
     *
     * @param speed
     * @param distance
     * @param timeout
     * @param useGyro
     * @param heading
     * @param aggressive
     * @throws InterruptedException
     */
    public void encoderDrive(double speed,
                             double distance,
                             double timeout,
                             boolean useGyro,
                             double heading,
                             boolean aggressive) throws InterruptedException {
        encoderDrive(speed, distance, timeout, useGyro, heading, aggressive, false, 0.0);
    }

    /**
     *
     * Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     *
     * @param speed                 Motor power (0 to 1.0)
     * @param distance              Inches
     * @param timeout               Seconds
     * @param useGyro               Use gyro to keep/curve to an absolute heading
     * @param heading               Heading to use
     * @throws InterruptedException
     */
    public void encoderDrive(double speed,
                             double distance,
                             double timeout,
                             boolean useGyro,
                             double heading,
                             boolean aggressive,
                             boolean userange,
                             double maintainRange) throws InterruptedException {

        // Calculated encoder targets
        int newLFTarget;
        int newRFTarget;
        int newLRTarget;
        int newRRTarget;

        // The potentially adjusted current target heading
        double curHeading = heading;

        // Speed ramp on start of move to avoid wheel slip
        final double MINSPEED = 0.30;           // Start at this power
        final double SPEEDINCR = 0.015;         // And increment by this much each cycle
        double curSpeed;                        // Keep track of speed as we ramp

        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {

            RobotLog.i("DM10337- Starting encoderDrive speed:" + speed +
                    "  distance:" + distance + "  timeout:" + timeout +
                    "  useGyro:" + useGyro + " heading:" + heading + "  maintainRange: " + maintainRange);

            // Calculate "adjusted" distance  for each side to account for requested turn during run
            // Purpose of code is to have PIDs closer to finishing even on curved moves
            // This prevents jerk to one side at stop
            double leftDistance = distance;
            double rightDistance = distance;
            if (useGyro) {
                // We are gyro steering -- are we requesting a turn while driving?
                double headingChange = getError(curHeading) * Math.signum(distance);
                if (Math.abs(headingChange) > 5.0) {
                    //Heading change is significant enough to account for
                    if (headingChange > 0.0) {
                        // Assume 16 inch wheelbase
                        // Add extra distance to the wheel on outside of turn
                        rightDistance += Math.signum(distance) * 2 * 3.1415 * 16 * headingChange / 360.0;
                        RobotLog.i("DM10337 -- Turn adjusted R distance:" + rightDistance);
                    } else {
                        // Assume 16 inch wheelbase
                        // Add extra distance from the wheel on inside of turn
                        // headingChange is - so this is increasing the left distance
                        leftDistance -= Math.signum(distance) * 2 * 3.1415 * 16 * headingChange / 360.0;
                        RobotLog.i("DM10337 -- Turn adjusted L distance:" + leftDistance);
                    }
                }
            }

            // Determine new target encoder positions, and pass to motor controller
            newLFTarget = robot.leftDrive1.getCurrentPosition() + (int)(leftDistance * robot.COUNTS_PER_INCH);
            newLRTarget = robot.leftDrive2.getCurrentPosition() + (int)(leftDistance * robot.COUNTS_PER_INCH);
            newRFTarget = robot.rightDrive1.getCurrentPosition() + (int)(rightDistance * robot.COUNTS_PER_INCH);
            newRRTarget = robot.rightDrive2.getCurrentPosition() + (int)(rightDistance * robot.COUNTS_PER_INCH);

            while(robot.leftDrive1.getTargetPosition() != newLFTarget){
                robot.leftDrive1.setTargetPosition(newLFTarget);
                sleep(1);
            }
            while(robot.rightDrive1.getTargetPosition() != newRFTarget){
                robot.rightDrive1.setTargetPosition(newRFTarget);
                sleep(1);
            }
            while(robot.leftDrive2.getTargetPosition() != newLRTarget){
                robot.leftDrive2.setTargetPosition(newLRTarget);
                sleep(1);
            }
            while(robot.rightDrive2.getTargetPosition() != newRRTarget){
                robot.rightDrive2.setTargetPosition(newRRTarget);
                sleep(1);
            }

            // Turn On motors to RUN_TO_POSITION
            robot.setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();

            speed = Math.abs(speed);    // Make sure its positive
            curSpeed = Math.min(MINSPEED,speed);

            // Set the motors to the starting power
            robot.leftDrive1.setPower(Math.abs(curSpeed));
            robot.rightDrive1.setPower(Math.abs(curSpeed));
            robot.leftDrive2.setPower(Math.abs(curSpeed));
            robot.rightDrive2.setPower(Math.abs(curSpeed));

            // keep looping while we are still active, and there is time left, until at least 1 motor reaches target
            while (opMode.opModeIsActive() &&
                    (runtime.seconds() < timeout) &&
                    robot.leftDrive1.isBusy() &&
                    robot.leftDrive2.isBusy() &&
                    robot.rightDrive1.isBusy() &&
                    robot.rightDrive2.isBusy()) {

                // Ramp up motor powers as needed
                if (curSpeed < speed) {
                    curSpeed += SPEEDINCR;
                }
                double leftSpeed = curSpeed;
                double rightSpeed = curSpeed;

                // Doing gyro heading correction?
                if (useGyro){

                    // adjust relative speed based on heading
                    double error = getError(curHeading);
                    double steer = getSteer(error,
                            (aggressive?P_DRIVE_COEFF_1:P_DRIVE_COEFF_2));

                    // if driving in reverse, the motor correction also needs to be reversed
                    if (distance < 0)
                        steer *= -1.0;

                    // Adjust motor powers for heading correction
                    leftSpeed -= steer;
                    rightSpeed += steer;

                    // Normalize speeds if any one exceeds +/- 1.0;
                    double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                    if (max > 1.0)
                    {
                        leftSpeed /= max;
                        rightSpeed /= max;
                    }

                }

                // And rewrite the motor speeds
                robot.leftDrive1.setPower(Math.abs(leftSpeed));
                robot.rightDrive1.setPower(Math.abs(rightSpeed));
                robot.leftDrive2.setPower(Math.abs(leftSpeed));
                robot.rightDrive2.setPower(Math.abs(rightSpeed));

                // Allow time for other processes to run.
                sleep(1);;
            }


            RobotLog.i("DM10337- encoderDrive done" +
                    "  lftarget: " +newLFTarget + "  lfactual:" + robot.leftDrive1.getCurrentPosition() +
                    "  lrtarget: " +newLRTarget + "  lractual:" + robot.leftDrive2.getCurrentPosition() +
                    "  rftarget: " +newRFTarget + "  rfactual:" + robot.rightDrive1.getCurrentPosition() +
                    "  rrtarget: " +newRRTarget + "  rractual:" + robot.rightDrive2.getCurrentPosition() +
                    "  heading:" + readGyro());

            // Stop all motion;
            robot.leftDrive1.setPower(0);
            robot.rightDrive1.setPower(0);
            robot.leftDrive2.setPower(0);
            robot.rightDrive2.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }


    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn (  double speed, double angle, double coefficient) {

        RobotLog.i("DM10337- gyroTurn start  speed:" + speed +
                "  heading:" + angle);

        // keep looping while we are still active, and not on heading.
        while (opMode.opModeIsActive() && !onHeading(speed, angle, coefficient)) {
            // Allow time for other processes to run.
            // onHeading() does the work of turning us
            sleep(1);;
        }

        RobotLog.i("DM10337- gyroTurn done   heading actual:" + readGyro());
    }


    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            // Close enough so no need to move
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            // Calculate motor powers
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        robot.leftDrive1.setPower(leftSpeed);
        robot.rightDrive1.setPower(rightSpeed);
        robot.leftDrive2.setPower(leftSpeed);
        robot.rightDrive2.setPower(rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;


        // calculate error in -179 to +180 range  (
        robotError = targetAngle - readGyro();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }


    /**
     * Record the current heading and use that as the 0 heading point for gyro reads
     * @return
     */
    void zeroGyro() {
        angles = robot.adaGyro.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        headingBias = angles.firstAngle;
    }


    /**
     * Read the current heading direction.  Use a heading bias if we recorded one at start to account for drift during
     * the init phase of match
     *
     * @return      Current heading (Z axis)
     */
    double readGyro() {
        angles = robot.adaGyro.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        return angles.firstAngle - headingBias;
    }

    /**
     * Always returns true as we are blue.
     *
     * Red OpMode would extend this class and Override this single method.
     *
     * @return          Always true
     */

    public int jewelColor() {
        if (robot.jewelCS.red() > robot.jewelCS.blue()) {
            return -1;
        } else {
            return 1;
        }
    }
    public int jewelHue() {

        // Return 1 for Blue and -1 for Red
        // convert the RGB adaValues to HSV adaValues.
        Color.RGBToHSV(robot.jewelCS.red() * 255, robot.jewelCS.green() * 255,
                robot.jewelCS.blue() * 255, adaHSV);

        // Normalize hue to -270 to 90 degrees
        while (adaHSV[0] >= 90.0) {
            adaHSV[0] -= 360.0;
        }
        while (adaHSV[0] < -270.0) {
            adaHSV[0] += 360.0;
        }


        opMode.telemetry.addData("Hue: ", adaHSV[0]);
        opMode.telemetry.update();
        // Check for blue
        if ((adaHSV[0] > BLUE_MIN) && (adaHSV[0] < BLUE_MAX)) {
            // we see blue so return 1.0
            RobotLog.i("DM10337- Jewel color found blue alpha. Hue:" + adaHSV[0] );
            return 1;
        }

        // Check for red
        if (adaHSV[0] > RED_MIN && adaHSV[0] < RED_MAX) {
            //telemetry.addData("beacon", -1);
            //telemetry.update();
            RobotLog.i("DM10337- Jewel color found red. Hue:" + adaHSV[0] );
            return -1;
        }

        RobotLog.i("DM10337- Jewel color found neither. Hue:" + adaHSV[0] );
        return 0;         // We didn't see either color so don't know
    }

    /**
     * Robot drives forward while attempting to collect glyph.
     * Robot attempts to maintain heading throughout collection procress
     * Robot stops once glyph is retrieved
     **/
    public void collectGlyph (double speed, int timeout, boolean useGyro, double heading) {

        robot.intake.setClosed();
        robot.intake.setIn();

        // The potentially adjusted current target heading
        double curHeading = heading;

        // Speed ramp on start of move to avoid wheel slip
        final double MINSPEED = 0.30;           // Start at this power
        final double SPEEDINCR = 0.015;         // And increment by this much each cycle
        double curSpeed;                        // Keep track of speed as we ramp

        // reset the timeout time and start motion.
        runtime.reset();

        speed = Math.abs(speed);    // Make sure its positive
        curSpeed = Math.min(MINSPEED,speed);

        // Set the motors to the starting power
        robot.leftDrive1.setPower(Math.abs(curSpeed));
        robot.rightDrive1.setPower(Math.abs(curSpeed));
        robot.leftDrive2.setPower(Math.abs(curSpeed));
        robot.rightDrive2.setPower(Math.abs(curSpeed));

        // keep looping while we are still active, and there is time left, until distance sensor detects glyph in intake

        boolean stop = false;

        while (opMode.opModeIsActive() && (runtime.seconds() < timeout) && !stop) {

            if ((robot.intake.distanceSensor_right.getDistance(DistanceUnit.CM) < 12.0) || (robot.intake.distanceSensor_right.getDistance(DistanceUnit.CM) < 12.0)) {
                stop = true;
            }

            // Ramp up motor powers as needed
            if (curSpeed < speed) {
                curSpeed += SPEEDINCR;
            }
            double leftSpeed = curSpeed;
            double rightSpeed = curSpeed;

            // Doing gyro heading correction?
            if (useGyro){

                // adjust relative speed based on heading
                double error = getError(curHeading);
                double steer = getSteer(error, P_DRIVE_COEFF_1);

                // stop trying to collect glyph if degree error is 5 or greater
                if (error >= 7.0) {
                    stop = true;
                }

                // if driving in reverse, the motor correction also needs to be reversed
                //if (distance < 0)
                //    steer *= -1.0;

                // Adjust motor powers for heading correction
                leftSpeed -= steer;
                rightSpeed += steer;

                // Normalize speeds if any one exceeds +/- 1.0;
                double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

            }

            // And rewrite the motor speeds
            robot.leftDrive1.setPower(Math.abs(leftSpeed));
            robot.rightDrive1.setPower(Math.abs(rightSpeed));
            robot.leftDrive2.setPower(Math.abs(leftSpeed));
            robot.rightDrive2.setPower(Math.abs(rightSpeed));

            // Allow time for other processes to run.
            sleep(1);;
        }

        robot.intake.intakeRightMotor.setPower(0.35);
        robot.intake.intakeLeftMotor.setPower(0.35);
        robot.leftDrive1.setPower(0.0);
        robot.leftDrive2.setPower(0.0);
        robot.rightDrive1.setPower(0.0);
        robot.rightDrive2.setPower(0.0);

    }

    /**
     * Robot returns to designatied encoder position
     **/

    public int determineDistance(int left1Pos, int left2Pos, int right1Pos, int right2Pos) {

        int averageNewPos = (robot.leftDrive1.getCurrentPosition() + robot.leftDrive2.getCurrentPosition() + robot.rightDrive1.getCurrentPosition() + robot.rightDrive2.getCurrentPosition()) / 4;
        int averageOldPos = (left1Pos + left2Pos + right1Pos + right2Pos) / 4;
        int difference = Math.abs(averageOldPos - averageNewPos);
        int inches = (int) (difference / robot.COUNTS_PER_INCH);
        return inches;
    }

    public void returnToPosition(double speed,
                                 int left1Pos,
                                 int left2Pos,
                                 int right1Pos,
                                 int right2Pos,
                                 double timeout,
                                 boolean useGyro,
                                 double heading) throws InterruptedException {


        // / The potentially adjusted current target heading
        double curHeading = heading;

        // Speed ramp on start of move to avoid wheel slip
        final double MINSPEED = 0.30;           // Start at this power
        final double SPEEDINCR = 0.015;         // And increment by this much each cycle
        double curSpeed;                        // Keep track of speed as we ramp

        int averageOriginalPos = (robot.leftDrive1.getCurrentPosition() + robot.leftDrive2.getCurrentPosition() + robot.rightDrive1.getCurrentPosition() + robot.rightDrive2.getCurrentPosition())/4;
        int averageNewPos = (left1Pos + left2Pos + right1Pos + right2Pos) / 4;
        int difference = averageNewPos - averageOriginalPos;

        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {


            while (robot.leftDrive1.getTargetPosition() != left1Pos) {
                robot.leftDrive1.setTargetPosition(left1Pos);
                sleep(1);
            }
            while (robot.rightDrive1.getTargetPosition() != right1Pos) {
                robot.rightDrive1.setTargetPosition(right1Pos);
                sleep(1);
            }
            while (robot.leftDrive2.getTargetPosition() != left2Pos) {
                robot.leftDrive2.setTargetPosition(left2Pos);
                sleep(1);
            }
            while (robot.rightDrive2.getTargetPosition() != right2Pos) {
                robot.rightDrive2.setTargetPosition(right2Pos);
                sleep(1);
            }

            // Turn On motors to RUN_TO_POSITION
            robot.setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();

            speed = Math.abs(speed);    // Make sure its positive
            curSpeed = Math.min(MINSPEED,speed);

            // Set the motors to the starting power
            robot.leftDrive1.setPower(Math.abs(curSpeed));
            robot.rightDrive1.setPower(Math.abs(curSpeed));
            robot.leftDrive2.setPower(Math.abs(curSpeed));
            robot.rightDrive2.setPower(Math.abs(curSpeed));

            // keep looping while we are still active, and there is time left, until at least 1 motor reaches target
            while (opMode.opModeIsActive() &&
                    (runtime.seconds() < timeout) &&
                    robot.leftDrive1.isBusy() &&
                    robot.leftDrive2.isBusy() &&
                    robot.rightDrive1.isBusy() &&
                    robot.rightDrive2.isBusy()) {

                // Ramp up motor powers as needed
                if (curSpeed < speed) {
                    curSpeed += SPEEDINCR;
                }
                double leftSpeed = curSpeed;
                double rightSpeed = curSpeed;

                // Doing gyro heading correction?
                if (useGyro){

                    // adjust relative speed based on heading
                    double error = getError(curHeading);
                    double steer = getSteer(error, P_DRIVE_COEFF_1);

                    // if driving in reverse, the motor correction also needs to be reversed
                    if (difference < 0)
                        steer *= -1.0;

                    // Adjust motor powers for heading correction
                    leftSpeed -= steer;
                    rightSpeed += steer;

                    // Normalize speeds if any one exceeds +/- 1.0;
                    double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                    if (max > 1.0)
                    {
                        leftSpeed /= max;
                        rightSpeed /= max;
                    }

                }

                // And rewrite the motor speeds
                robot.leftDrive1.setPower(Math.abs(leftSpeed));
                robot.rightDrive1.setPower(Math.abs(rightSpeed));
                robot.leftDrive2.setPower(Math.abs(leftSpeed));
                robot.rightDrive2.setPower(Math.abs(rightSpeed));

                // Allow time for other processes to run.
                sleep(1);;
            }


            RobotLog.i("DM10337- encoderDrive done" +
                    "  lftarget: " +left1Pos + "  lfactual:" + robot.leftDrive1.getCurrentPosition() +
                    "  lrtarget: " +left2Pos+ "  lractual:" + robot.leftDrive2.getCurrentPosition() +
                    "  rftarget: " +right1Pos+ "  rfactual:" + robot.rightDrive1.getCurrentPosition() +
                    "  rrtarget: " +right2Pos+ "  rractual:" + robot.rightDrive2.getCurrentPosition() +
                    "  heading:" + readGyro());

            // Stop all motion;
            robot.leftDrive1.setPower(0);
            robot.rightDrive1.setPower(0);
            robot.leftDrive2.setPower(0);
            robot.rightDrive2.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    public void secureGlyph() {

        robot.intake.setInAlt();

        sleep (250);

        robot.intake.setStop();

        sleep (250);

        robot.intake.setIn();

        sleep (250);

        robot.intake.intakeRightMotor.setPower(0.35);
        robot.intake.intakeLeftMotor.setPower(0.35);
    }

    public void autoLoadFirstGlyph() {

        robot.lift.setLiftBtm();

        while (!robot.lift.reachedFloor()) sleep(1);;

        robot.gripper.setBtmClosed();

        while (robot.gripper.btmIsMoving()) sleep(1);;

        robot.intake.setOpen();

        while (robot.intake.isMoving()) sleep(1);;

        robot.lift.setLiftTop();

        while (robot.lift.distFromBottom() < 9.0) sleep(1);;

        robot.gripper.flip();

        while (robot.gripper.isFlipping()) sleep(1);;

        robot.lift.setLiftHeight(7.0);
    }

    public void autoLoadSecondGlyph() {

        robot.lift.setLiftBtm();

        while (!robot.lift.reachedFloor()) sleep(1);;

        robot.gripper.setBtmClosed();

        while (robot.gripper.btmIsMoving()) sleep(1);;

        robot.intake.setOpen();

        while (robot.intake.isMoving()) sleep(1);;

        robot.lift.setLiftHeight(8.0);
    }

    public boolean waitForSwitch() {
        while (!opMode.gamepad1.a) {
            sleep(1);;
        }
        return true;
    }

    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    boolean iAmBlue() {return weAreBlue;}
}
