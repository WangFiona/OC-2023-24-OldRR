package overcharged.components;

import static overcharged.config.RobotConstants.TAG_H;
import static overcharged.config.RobotConstants.TAG_SL;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.ArrayList;
import java.util.List;

import overcharged.config.RobotConstants;

public class vSlides {

    public OcMotorEx vSlides;
    public OcSwitch switchSlideDown;
    public final List<OcSwitch> switchs = new ArrayList<>();

    public double start;

    public static int autoLevel = 450;
    public static int level1 = 805;
    public static int level2 = 1225;
    public static int level3 = 1700;
    public static int level4 = 2000;

    public static double p = 16;
    public static double i = 0;
    public static double d = 0;
    public static double f = 0;

    public vSlides(HardwareMap hardwareMap){
        vSlides = new OcMotorEx(hardwareMap, "vSlides", DcMotor.Direction.FORWARD, DcMotor.RunMode.RUN_USING_ENCODER);
        vSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        vSlides.setTargetPositionPIDFCoefficients(p, i, d, f);

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
        start = vSlides.getCurrentPosition();

        RobotLog.ii(TAG_SL, "Initialized the Slide component numberMissing=" + numberMissing + " missing=" + missing);
    }

    public void up(){
        vSlides.setPower(1);
    }

    public void off(){
        vSlides.setPower(0);
    }

    public void down(){
        vSlides.setPower(-1f);
    }

    public void moveEncoderTo(int pos, float power){
        vSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        vSlides.setTargetPosition(pos);
        vSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        vSlides.setPower(power);
    }

    public boolean slideReachedBottom() {
        if (switchSlideDown.isDisabled()) return vSlides.getCurrentPosition() <= start;
        return switchSlideDown.isTouch();
    }
    public void setPower(float power){vSlides.setPower(power);}

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
}
