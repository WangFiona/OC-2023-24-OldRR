package overcharged.components;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class newDepo {

    public OcServo frontClaw;
    public OcServo backClaw;
    public OcServo wrist;
    public OcServo arm;

    public static float FRONT_CLOSE = 100f;
    public static float FRONT_OPEN = 150f;
    public static float BACK_CLOSE = 100f;
    public static float BACK_OPEN = 150f;

    public static float WRIST_FLAT = 100f;
    public static float WRIST_P_DIAG= 50f;
    public static float WRIST_N_DIAG= 150f;

    public static float ARM_IN = 100f;
    public static float ARM_OUT = 150f;

    public newDepo(HardwareMap hardwareMap) {
        frontClaw = new OcServo(hardwareMap, "frontClaw", FRONT_OPEN);
        backClaw = new OcServo(hardwareMap, "backClaw", BACK_OPEN);
        wrist = new OcServo(hardwareMap, "wrist", WRIST_FLAT);
        arm = new OcServo(hardwareMap, "arm", ARM_IN);

    }
    public void setFrontClawPos(float pos){
        frontClaw.setPosition(pos);
    }

    public void setBackClawPos(float pos){
        backClaw.setPosition(pos);
    }

    public void setWristPos(float pos){
        wrist.setPosition(pos);
    }

    public void setArmPos(float pos){
        arm.setPosition(pos);
    }
}

