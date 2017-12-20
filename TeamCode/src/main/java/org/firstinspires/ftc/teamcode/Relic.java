

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 *    Everything related to the lift motor
 */
public class Relic {

    // Hardware
    public DcMotor relicMotor = null;
    public Servo relicGrip = null;
    public Servo relicPivot = null;


    // Servo Constants
    static final double RELIC_GRIP_OPEN =  0.0;
    static final double RELIC_GRIP_CLOSE = 1.0;
    static final double RELIC_PIVOT_HOME = 0.0;
    static final double RELIC_PIVOT_OUT = 1.0;

    /* Lift constants */
    static final double     RELIC_POWER = 1.0;
    static final int        EXTENSION_COUNTS_PER_MOTOR_REV    = 7 ;    // Neverrest
    static final double     EXTENSION_DRIVE_GEAR_REDUCTION    = 20; // Neverest 20:1
    static final double     EXTENSION_PULLEY_DIAMETER_INCHES   = 3.0;     // For figuring circumference
    static final double     EXTENSION_COUNTS_PER_INCH         = (4 * EXTENSION_COUNTS_PER_MOTOR_REV * EXTENSION_DRIVE_GEAR_REDUCTION) /
            (EXTENSION_PULLEY_DIAMETER_INCHES * 3.1415);


    // Lift variables
    public final int RELIC_OUT_POS = (int) (36*EXTENSION_COUNTS_PER_INCH);
    public final int RELIC_IN_POS = (int) (1*EXTENSION_COUNTS_PER_INCH);

    /**
     * Constructor
     */
    public void Relic() {
        // Do nothing
    }

    /**
     *  Initialize the relic
     *
     * @param hw    Hardwaremap for our robot
     * @param motor relic extend motor
     * @param servoGrip relic grip servo
     * @param servoPivot relic pivot servo
     */

    public void init (HardwareMap hw, String motor, String servoGrip, String servoPivot) {
        // Define and Initialize intake Motors
        relicMotor = hw.dcMotor.get(motor);
        relicMotor.setDirection(DcMotor.Direction.FORWARD);
        relicMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Tune the motor PID parameters
        if (relicMotor instanceof DcMotorEx) {
            DcMotorEx theLift = (DcMotorEx) relicMotor;
            PIDCoefficients pid = theLift.getPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
            // Do any needed PID value adjustments here
            theLift.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pid);

        }

        // Set their operating modes and stop them
        relicMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        relicMotor.setPower(0.0);

        // Define and Initialize servos
        relicGrip = hw.servo.get(servoGrip);
        relicGrip.setPosition(RELIC_GRIP_CLOSE);
        relicPivot = hw.servo.get(servoPivot);
        relicPivot.setPosition(RELIC_PIVOT_HOME);
    }

    // Set relic arm to OUT position
    public void setRelicExtensionOut() {
        relicMotor.setTargetPosition(RELIC_OUT_POS);
        relicMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        relicMotor.setPower(RELIC_POWER);
    }

    // Set relic arm to IN position
    public void setRelicExtensionIn() {
        relicMotor.setTargetPosition(RELIC_IN_POS);
        relicMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        relicMotor.setPower(RELIC_POWER);
        }

    // Hard Stop relic
    public void stopRelicExtension() {
        relicMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        relicMotor.setPower(0.0);
    }

    // Set relic grip open
    public void setRelicGripOpen() { relicGrip.setPosition(RELIC_GRIP_OPEN);}

    // Set relic grip closed
    public void setRelicGripClose() { relicGrip.setPosition(RELIC_GRIP_CLOSE);}

    // Set relic pivot to home position
    public void setRelicPivotHome() { relicPivot.setPosition(RELIC_PIVOT_HOME);}

    // Set relic pivot to out position
    public void setRelicPivotOut() { relicPivot.setPosition(RELIC_PIVOT_OUT);}

    // Manually rotate relic pivot
    public void rotate(double speed) {
        speed = Range.clip(speed, -1, 1);
        double current = relicPivot.getPosition();
        double target =  current + (((RELIC_PIVOT_OUT - RELIC_PIVOT_HOME) * speed) / 4);   // At full stick will take 20 cycles
        target = Range.clip(target, RELIC_PIVOT_OUT, RELIC_PIVOT_HOME);
        relicPivot.setPosition(target);
    }

    public double getExtensionDistance() {
        return (relicMotor.getCurrentPosition()/EXTENSION_COUNTS_PER_INCH);
    }
}


