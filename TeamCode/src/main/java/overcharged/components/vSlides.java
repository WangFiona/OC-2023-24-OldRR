package overcharged.components;

import static overcharged.config.RobotConstants.TAG_H;
import static overcharged.config.RobotConstants.TAG_SL;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.ArrayList;
import java.util.List;

import overcharged.config.RobotConstants;
import overcharged.util.PIDCalculator;

public class vSlides {
    ///Overcharged mecanum robot
    private RobotMecanum robot;
    //Slide motors
    public final OcMotorEx vSlidesF;
    public final OcMotorEx vSlidesB;
    public OcSwitch switchSlideDown;
    public final List<OcSwitch> switchs = new ArrayList<>();

    //Slide level encoder values
    /*public static int level4 = 1970;
    public static int level3 = 1470;
    public static int level2 = 880;*/
    public static int autoLevel = 500;
    public static int level1 = 805;
    public static int level2 = 1225;
    public static int level3 = 1700;
    public static int level4 = 2000;
    public static float factorR = 1;//0.957f;//0.963f;//1.015f;
    public static float factorL = 1f;
    //starting encoder reading
    public double start;
    ///Disable the slides if the switch says touched even at this encoder level of the slides
    public static int SLIDE_DISABLE_AT = 200;
    ///Slowdown the slide motor at this encoder level of the slides
    public static int SLIDE_SLOW_AT = 200;
    ///Speedup the slide motor at this encoder level of the slides
    public static int SLIDE_MOREPOWER_AT = 80;
    public static int max_level_value = level4+100; //lesser value of the two motors. Use tester to get this value

    ///Slide power constant
    float SLIDE_POWER_UP_PID = 1f;
    float SLIDE_POWER_DOWN_PID = -0.8f;
    public static float SLIDE_POWER_UP = 0.8f;
    public static float SLIDE_POWER_DOWN = -0.85f;
    public static float SLIDE_POWER_DOWN_OUT = -0.3f;
    private static float SLIDE_POWER_DOWN_MIN = -0.15f;
    static int SLIDE_POSITION_THRESHOLD = 5;

    public int currentPositionL = 0;
    public int currentPositionR = 0;

    /**
     * The state of the Slide
     */
    public enum State {
        UP,
        DOWN,
        STOP
    }

    ///Maintain the global slide state
    public State state = State.DOWN;
    ///Maintain the global slide previous state
    private State prev_state = State.DOWN;
    ///Initialize the PIDCalculator
    public static double p = 0.3;
    public static double i = 0;
    public static double d = 0;
    private PIDCalculator pidController = new PIDCalculator(p, i, d);
    ///maintain the previous state of the slides PID or not
    private boolean pidState = false;
    //private Arm arm;


    /**
     * Initialize the slide system
     */
    public vSlides(HardwareMap hardwareMap) {
        ///Initialize slide motors
        String missing = "";
        int numberMissing = 0;
        OcMotorEx slideF = null;
        try {
            slideF = new OcMotorEx(hardwareMap,
                    "slideL",
                    DcMotor.Direction.FORWARD, DcMotor.RunMode.RUN_USING_ENCODER);
        } catch (Exception e) {
            RobotLog.ee(RobotConstants.TAG_R, "missing: slideL " + e.getMessage());
            missing = missing + ", slideL";
            numberMissing++;
        }
        this.vSlidesF = slideF;

        OcMotorEx slideB = null;
        try {
            slideB = new OcMotorEx(hardwareMap,
                    "slideR",
                    DcMotor.Direction.REVERSE, DcMotor.RunMode.RUN_USING_ENCODER);
        } catch (Exception e) {
            RobotLog.ee(RobotConstants.TAG_R, "missing: slideR " + e.getMessage());
            missing = missing + ", slideR";
            numberMissing++;
        }
        this.vSlidesB = slideB;
        OcSwitch vswitch = null;
        try {
            vswitch = new OcSwitch(hardwareMap,"vlimitswitch", true);
            boolean isSwitchNull= switchSlideDown==null ? true : false;
            switchs.add(vswitch);
            RobotLog.ii(TAG_H, "limitSwitch(isNull)? " + isSwitchNull);
        } catch (Exception e) {
            RobotLog.ee(RobotConstants.TAG_R,  "missing: limitSwitch " + e.getMessage());
            missing = missing + ", vlimitswitch";
            numberMissing++;
            boolean isSwitchNull= switchSlideDown==null ? true : false;
            RobotLog.ii(TAG_H, "limitSwitch(catch)? " + isSwitchNull);
        }
        this.switchSlideDown = vswitch;
        initialize(this.vSlidesF);
        initialize(this.vSlidesB);
        resetSlidePosition();
        RobotLog.ii(TAG_SL, "Initialized the Slide component numberMissing=" + numberMissing + " missing=" + missing);
        RobotLog.ii(TAG_SL, "Initialized the Slide component slideLeft=" + vSlidesF.getCurrentPosition() + " slideRight=" + vSlidesB.getCurrentPosition());
    }

    public boolean slideReachedBottom() {
        if (switchSlideDown.isDisabled()) return vSlidesF.getCurrentPosition() <= start;
        return isSlideSwitchPressed();
    }

    public void resetSlidePosition(){
        currentPositionL = 0;
        currentPositionR = 0;
        start = vSlidesF.getCurrentPosition();
    }

    public double getCurrentPosition(){
        return vSlidesF.getCurrentPosition();
    }

    public boolean isSlideSwitchPressed() {
        return switchSlideDown.isTouch();
    }

    private void initialize(OcMotorEx motor) {
        reset(motor);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void reset(OcMotorEx motor) {
        motor.setPower(0f);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.resetPosition();
    }

    /**
     * Check if the slide reached the max upper limit using encoders
     *
     * @return
     */
    private boolean reachedMaxUpperLimit() {
        int positionL = vSlidesF.getCurrentPosition();
        int positionR = vSlidesB.getCurrentPosition();
        RobotLog.ii(TAG_SL, "reachedMaxUpperLimit() positionL=" + positionL + " positionR=" + positionR + " max_level_value=" + max_level_value);
        if (positionL > max_level_value || positionR > max_level_value) {
            RobotLog.ii(TAG_SL, "Reached max level positionL=" + positionL + " positionR=" + positionR + " max_level_value=" + max_level_value);
            return true;
        }
        return false;
    }

    ///	 Get the encoder value for the requested level, to move to
    public int getDistanceL(int pos)
    {
        int positionL = vSlidesF.getCurrentPosition();
        //calculate the distance to travel using the left side slide motor
        int distance = pos - positionL;
        return distance;
    }

    ///	 Get the encoder value for the requested level, to move to
    public int getDistanceR(int pos)
    {
        int positionR = vSlidesB.getCurrentPosition();
        //calculate the distance to travel using the left side slide motor
        int distance = pos - positionR;
        return distance;
    }

    /**
     * Move the slides to the last bottom level
     */
    public void moveToBottom()
    {
        //moveSlidesTo( 0);
        vSlidesB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        vSlidesF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        move(SLIDE_POWER_DOWN);
    }

    /**
     * @return the power from the PID calculations
     */
    public double getPidPower(int position, int level_value) {
        int wantedPos = position + level_value;
        int error = wantedPos - position;
        return pidController.getPID(error);
    }

    ///Is the slide at the position where we want to check for switch is disabled
    private boolean isAtDisablePosition()
    {
        int positionL = vSlidesF.getCurrentPosition();
        int positionR = vSlidesB.getCurrentPosition();
        if (positionL > SLIDE_DISABLE_AT || positionR > SLIDE_DISABLE_AT) return true;
        return false;
    }

    ///Is the slide at the position where we want to slow down the slide motors
    private boolean isAtLowPosition()
    {
        int positionL = vSlidesF.getCurrentPosition();
        int positionR = vSlidesB.getCurrentPosition();
        RobotLog.ii(TAG_SL, "isAtLowPosition positionL=" + positionL + " positionR=" + positionR + " SLIDE_SLOW_AT=" + SLIDE_SLOW_AT);
        if (positionL < SLIDE_SLOW_AT || positionR < SLIDE_SLOW_AT) return true;
        return false;
    }

    ///Is the slide almost at the bottom position where we want to more power to tighten and bring down the slide
    private boolean isAlmostBottomPosition()
    {
        int positionL = vSlidesF.getCurrentPosition();
        int positionR = vSlidesB.getCurrentPosition();
        RobotLog.ii(TAG_SL, "isAlmostBottomPosition positionL=" + positionL + " positionR=" + positionR + " SLIDE_MOREPOWER_AT=" + SLIDE_MOREPOWER_AT);
        if (positionL < SLIDE_MOREPOWER_AT || positionR < SLIDE_MOREPOWER_AT) return true;
        return false;
    }

    /**
     * Keep the slide system at the current level at a specified power using PID
     */
    public void keep() {
        if (pidState) return;
        state = State.STOP;
        if (prev_state == state) return;
        prev_state = state;
        vSlidesF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        vSlidesB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int positionL = vSlidesF.getCurrentPosition();
        int positionR = vSlidesB.getCurrentPosition();
        RobotLog.ii(TAG_SL, "Keep the slide at current level positionL=" + positionL + " positionR=" + positionR);
        float powerL = (float)(pidController.getPID(positionL));
        float powerR = (float)(pidController.getPID(positionR));
        vSlidesF.setTargetPosition(positionL);
        vSlidesB.setTargetPosition(positionR);
        currentPositionL = positionL;
        currentPositionR = positionR;
        vSlidesF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        vSlidesB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        vSlidesF.setPower(powerL);
        vSlidesB.setPower(powerR);
    }

    private boolean isInRange(int x, int y)
    {
        if (Math.abs(x-y) < SLIDE_POSITION_THRESHOLD) {
            return true;
        }
        return false;
    }

    /**
     * move the slide system up/down to the specified level at a calculated power using PID
     * @param pos encoder value of the level we want to reach
     */
    public void moveEncoderTo(int pos, int p) {
        pidState = true;
        vSlidesF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        vSlidesB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        int positionL = vSlidesF.getCurrentPosition();
        int positionR = vSlidesB.getCurrentPosition();
        int distanceL = getDistanceL(pos);
        int distanceR = getDistanceR(pos);
        if (Math.signum(distanceL) > 0) {
            state = State.UP;
            if (prev_state != state) prev_state = state;
            RobotLog.ii(TAG_SL, "Move the slide level up to " + pos);
            if (reachedMaxUpperLimit()) {
                RobotLog.ii(TAG_SL, "Slide reached max level so quitting");
                setPower(0f);
                return;
            }
            RobotLog.ii(TAG_SL, "vSlides: Slide up positionL=" + positionL + " positionR=" + positionR + " SLIDE_DISABLE_AT=" + SLIDE_DISABLE_AT);
            if (isAtDisablePosition() && (switchSlideDown != null) &&
                    !switchSlideDown.isDisabled() && isSlideSwitchPressed()) {
                /// If the slide down switch is still showing as on/isTouch() then there is a problem with the switch (not connected or broken)
                /// So disable it to allow slide down functionality
                RobotLog.ii(TAG_SL, "Slide up switchSlideDown.disable()");
                switchSlideDown.disable();
                //if (robot != null) robot.ledRedBlink();
            }
        } else {
            state = State.DOWN;
            if (prev_state != state) prev_state = state;
            RobotLog.ii(TAG_SL, "Move the slide level down to " + pos);
            if (!switchSlideDown.isDisabled() && isSlideSwitchPressed()) {
                reset(vSlidesF);
                reset(vSlidesB);
                resetSlidePosition();
                RobotLog.ii(TAG_SL, "Slide switch touched");
                return;
            }
        }
        int gotoPositionL = positionL + distanceL;
        if (gotoPositionL < 0) gotoPositionL = 0;
        int gotoPositionR = positionR + distanceR;
        if (gotoPositionR < 0) gotoPositionR = 0;
        float power = (float)(getPower(vSlidesF, pos));
        RobotLog.ii(TAG_SL, "vSlides: Move the slide to position='" + pos + "' distanceL='" + distanceL + "' distanceR='" + distanceR  + "' gotoPositionL='" + gotoPositionL  + "' gotoPositionR='" + gotoPositionR  + "' power='" + power + "'");
        vSlidesF.setTargetPosition((int)(factorL*gotoPositionL));
        vSlidesB.setTargetPosition((int)(factorR*gotoPositionR));
        currentPositionL = gotoPositionL;
        currentPositionR = gotoPositionR;
        vSlidesF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        vSlidesB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        vSlidesF.setPower(power);
        vSlidesB.setPower(power);
        RobotLog.ii(TAG_SL, "powerL " + vSlidesF.getPower() + " powerR " + vSlidesB.getPower());
    }

    public double getPower(OcMotorEx slide, double gotoPosition) {
        double currentPosition = slide.getCurrentPosition();
        double distance = gotoPosition - currentPosition;
        float power = Math.signum(distance) < 0 ? SLIDE_POWER_DOWN_PID: SLIDE_POWER_UP_PID;
        RobotLog.ii(TAG_SL, "getPower sign=" + Math.signum(distance) + " gotoPosition='" + gotoPosition +  "' currentPosition='" + currentPosition + "' power='" + power + "'");
        return power;
    }

    /**
     * turn the slides system at a specified power and direction until it reaches the predetermined maximum
     *
     * @param up when true moves the slide up. false moves the slide down
     * @param powerFactor the power factor, using this value make it slower or full speed
     */
    private void moveSlides(boolean up, float powerFactor) {
        pidState = false;
        vSlidesF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        vSlidesB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lastPowerFactor = powerFactor;
        if (up) {
            RobotLog.ii(TAG_SL, "Move the slide up powerFactor=" + powerFactor);
            state = State.UP;
            if (reachedMaxUpperLimit()) {
                RobotLog.ii(TAG_SL, "Slide reached max level so quitting");
                setPower(0f);
                return;
            }
            setPower(powerFactor * SLIDE_POWER_UP);
            if (prev_state != state) prev_state = state;
            if (isAtDisablePosition() && (switchSlideDown != null) &&
                    !switchSlideDown.isDisabled() && isSlideSwitchPressed()) {
                /// If the slide down switch is still showing as on/isTouch() then there is a problem with the switch (not connected or broken)
                /// So disable it to allow slide down functionality
                RobotLog.ii(TAG_SL, "Slide up switchSlideDown.disable()");
                switchSlideDown.disable();
                //if (robot != null) robot.ledRedBlink();
            }
        } else {
            RobotLog.ii(TAG_SL, "Move the slide down powerFactor=" + powerFactor);
            state = State.DOWN;
            if (prev_state != state) prev_state = state;
            if (!slideReachedBottom()) {
                setPower(getSlideDownPower(powerFactor));
            } else {
                RobotLog.ii(TAG_SL, "switchSlideDown is touched and slide left position=" + vSlidesF.getCurrentPosition());
                reset(vSlidesF);
                reset(vSlidesB);
                resetSlidePosition();
            }
        }
    }

    private float getSlideDownPower(float powerFactor) {
        float power = powerFactor * SLIDE_POWER_DOWN;
        RobotLog.ii(TAG_SL, "getSlideDownPower=" + power);
        return power;
    }

    /**
     * Move the slides up/down
     * if y > 0 the slide moves up
     * if y < 0 the slide moves down
     */
    public void move(float y)
    {
        RobotLog.ii(TAG_SL, "move slide y " + y);
        if (y == 0) return;
        boolean up = (y > 0) ? true : false;
        if (up) {
            moveSlides(true, y);
        } else {
            moveSlides(false,Math.abs(y));
        }
    }

    /**
     * Save the last power used so that we can use it to perform some automation.
     * Use it to keep the slides stationary up/down
     */
    private float lastPowerFactor = 0f;

    /**
     * stop the slide motors
     */
    public void stop()
    {
        if (pidState) return;
        if (prev_state != State.STOP) {
            RobotLog.ii(TAG_SL, "Stop the Slide component");
            setPower(0f);
            prev_state = State.STOP;
        }
        state = State.STOP;
        lastPowerFactor = 0f;
    }

    /**
     * stop the slide motors
     */
    public void forcestop()
    {
        //if (prev_state != State.STOP) {
        RobotLog.ii(TAG_SL, "Force stop the Slide component");
        setPower(0f);
        prev_state = State.STOP;
        //}
        state = State.STOP;
        lastPowerFactor = 0f;
        RobotLog.ii(TAG_SL, "powerL " + vSlidesF.getPower() + " powerR " + vSlidesB.getPower());
    }

    /**
     * Set the power on both the motors. Also accommodate and report for any motor failures.
     * @param power power to set
     */
    public void setPower(float power)
    {
        int cnt = 1;
        //try {
        if (vSlidesF != null) {
            RobotLog.ii(TAG_SL, "Set slide left motor power to " + power);
            vSlidesF.setPower(power);
            if (power == 0f) {
                vSlidesF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
        } else {
            RobotLog.ii(TAG_SL, "Not setting power for motorL");
        }
        cnt = 2;
        if (vSlidesB != null) {
            RobotLog.ii(TAG_SL, "Set slide right motor power to " + power);
            vSlidesB.setPower(power);
            if (power == 0f) {
                vSlidesB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
        } else {
            RobotLog.ii(TAG_SL, "Not setting power for motorR");
        }
    }

    public void stopMotors(){
        vSlidesF.setPower(0f);
        vSlidesB.setPower(0f);
    }

    public void down(){
        vSlidesF.setPower(-1f);
        vSlidesB.setPower(-1f);
    }
}

/*public class vSlides {

    public OcMotorEx vSlidesF;
    public OcMotorEx vSlidesB;
    public OcSwitch switchSlideDown;
    public final List<OcSwitch> switchs = new ArrayList<>();

    public double start;

    public static int autoLevel = 500;
    public static int level1 = 805;
    public static int level2 = 1225;
    public static int level3 = 1700;
    public static int level4 = 2000;

    public static double p = 16;
    public static double i = 0;
    public static double d = 0;
    public static double f = 0;

    public vSlides(HardwareMap hardwareMap){
        vSlidesB = new OcMotorEx(hardwareMap, "vSlidesF", DcMotor.Direction.FORWARD, DcMotor.RunMode.RUN_USING_ENCODER);
        vSlidesB = new OcMotorEx(hardwareMap, "vSlidesB", DcMotor.Direction.FORWARD, DcMotor.RunMode.RUN_USING_ENCODER);
        vSlidesB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        vSlidesB.setTargetPositionPIDFCoefficients(p, i, d, f);

        String missing = "";
        int numberMissing = 0;
        OcMotorEx slideL = null;
        OcSwitch lswitch = null;
        try {
            lswitch = new OcSwitch(hardwareMap,"limitswitch", true);
            boolean isSwitchNull= switchSlideDown==null ? true : false;
            switchs.add(lswitch);
            RobotLog.ii(TAG_H, "limitSwitch(isNull)? " + isSwitchNull);
        } catch (Exception e) {
            RobotLog.ee(RobotConstants.TAG_R,  "missing: limitSwitch " + e.getMessage());
            missing = missing + ", limitswitch";
            numberMissing++;
            boolean isSwitchNull= switchSlideDown==null ? true : false;
            RobotLog.ii(TAG_H, "limitSwitch(catch)? " + isSwitchNull);
        }
        this.switchSlideDown = lswitch;
        start = vSlidesB.getCurrentPosition();

        RobotLog.ii(TAG_SL, "Initialized the Slide component numberMissing=" + numberMissing + " missing=" + missing);
    }

    public void up(){
        vSlidesB.setPower(1);
    }

    public void off(){
        vSlidesB.setPower(0);
    }

    public void down(){
        vSlidesB.setPower(-1f);
    }

    public void moveEncoderTo(int pos, float power){
        vSlidesB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        vSlidesB.setTargetPosition(pos);
        vSlidesB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        vSlidesB.setPower(power);
    }

    public boolean slideReachedBottom() {
        if (switchSlideDown.isDisabled()) return vSlidesB.getCurrentPosition() <= start;
        return switchSlideDown.isTouch();
    }
    public void setPower(float power){
        vSlidesB.setPower(power);}

    public void reset(OcMotorEx motor) {
        motor.setPower(0f);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.resetPosition();
    }

    public void forceStop() {
        //if (prev_state != State.STOP) {
        RobotLog.ii(TAG_SL, "Force stop the Slide component");
        setPower(0f);
    }
}*/
