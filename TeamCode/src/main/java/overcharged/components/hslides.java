package overcharged.components;

import static overcharged.config.RobotConstants.TAG_H;
import static overcharged.config.RobotConstants.TAG_SL;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.ArrayList;
import java.util.List;

import overcharged.config.RobotConstants;

public class hslides {

    public OcMotorEx hslides;
    public OcSwitch switchSlideDown;
    public final List<OcSwitch> switchs = new ArrayList<>();

    public double start;

    public static int autoLevel = 0;

    public static final int START = 0;
    public static final int PRESET1 = 142;
    public static final int OUT = 1000;

    public static double p = 18;
    public static double i = 0;
    public static double d = 0;
    public static double f = 0;

    public hslides(HardwareMap hardwareMap){
        hslides = new OcMotorEx(hardwareMap, "hSlides", DcMotor.Direction.FORWARD, DcMotor.RunMode.RUN_USING_ENCODER);
        hslides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hslides.setTargetPositionPIDFCoefficients(p, i, d, f);

        String missing = "";
        int numberMissing = 0;
        OcMotorEx slideL = null;
        OcSwitch lswitch2 = null;
        try {
            lswitch2 = new OcSwitch(hardwareMap,"hlimitswitch", true);
            boolean isSwitchNull= switchSlideDown==null ? true : false;
            switchs.add(lswitch2);
            RobotLog.ii(TAG_H, "limitSwitch(isNull)? " + isSwitchNull);
        } catch (Exception e) {
            RobotLog.ee(RobotConstants.TAG_R,  "missing: limitSwitch " + e.getMessage());
            missing = missing + ", hlimitswitch";
            numberMissing++;
            boolean isSwitchNull= switchSlideDown==null ? true : false;
            RobotLog.ii(TAG_H, "limitSwitch(catch)? " + isSwitchNull);
        }
        this.switchSlideDown = lswitch2;
        start = hslides.getCurrentPosition();

        RobotLog.ii(TAG_SL, "Initialized the Slide component numberMissing=" + numberMissing + " missing=" + missing);
    }

    public void out(){
        hslides.setPower(1);
    }

    public void off() { hslides.setPower(0);
    }

    public void in(){
        hslides.setPower(-1f);
    }

    public void moveEncoderTo(int pos, float power){
        hslides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hslides.setTargetPosition(pos);
        hslides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hslides.setPower(power);
    }

    public boolean slideIn() {
        if (switchSlideDown.isDisabled()) return hslides.getCurrentPosition() <= start;
        return switchSlideDown.isTouch();
    }
    public void setPower(float power){hslides.setPower(power);}

    public void reset(OcMotorEx motor) {
        motor.setPower(0f);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.resetPosition();
    }

    public float getPower() {
        return hslides.getPower();
    }

    public void forceStop() {
        //if (prev_state != State.STOP) {
        RobotLog.ii(TAG_SL, "Force stop the Slide component");
        setPower(0f);
    }
}

