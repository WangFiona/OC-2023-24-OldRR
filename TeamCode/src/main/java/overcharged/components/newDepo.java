package overcharged.components;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class newDepo {

    public OcServo frontClaw;
    public OcServo backClaw;
    public OcServo wrist;
    public OcServo arm;
    public AnalogInput armVolt;

    public static float FRONT_CLOSE = 7f;
    public static float FRONT_DUMP = 151f;
    //public static float FRONT_OPEN = 151f;
    public static float BACK_CLOSE = 250f;
    public static float BACK_DUMP = 98f;
    //public static float BACK_OPEN = 98f;

    public static float WRIST_IN_VERT = 146f;//53f;
    public static float WRIST_OPP_VERT = 10f;//237f;
    public static float WRIST_OPP_FLAT = 57f;//142f;
    public static float WRIST_FLAT = 237f;//142f;
    public static float WRIST_R_DIAG= 117f;//107f;
    public static float WRIST_L_DIAG = 181f;//88f;

    /*public static float RIGHT_IN = 80f;
    public static float LEFT_IN = 183f;

    public static float R_OUT_VERT;
    public static float L_OUT_VERT;
    public static float R_OUT_FLAT;
    public static float L_OUT_FLAT;*/

    public static float ARM_IN = 174f;
    public static float ARM_OUT = 70f;

    public newDepo(HardwareMap hardwareMap) {
        frontClaw = new OcServo(hardwareMap, "frontClaw", FRONT_DUMP);
        backClaw = new OcServo(hardwareMap, "backClaw", BACK_DUMP);
        wrist = new OcServo(hardwareMap, "wrist", WRIST_IN_VERT);
        arm = new OcServo(hardwareMap, "arm", ARM_IN);
        armVolt = hardwareMap.get(AnalogInput.class, "armVolt");
    }
    public void setFrontClawPos(float pos){
        frontClaw.setPosition(pos);
    }

    public void setBackClawPos(float pos){
        backClaw.setPosition(pos);
    }

    public void setBothClawsOpen(){
        frontClaw.setPosition(FRONT_DUMP);
        backClaw.setPosition(BACK_DUMP);
    }

    public void setBothClawsClose(){
        frontClaw.setPosition(FRONT_CLOSE);
        backClaw.setPosition(BACK_CLOSE);
    }

    public void setWristPos(float pos){
        wrist.setPosition(pos);
    }

    public void setArmPos(float pos){
        arm.setPosition(pos);
    }

    public double getArmVolt(){
        return armVolt.getVoltage(); //0 to 3.3
    }

    /*public void setDepoOutVert(){
        wrist.setPosition(R_OUT_VERT);
        arm.setPosition(L_OUT_VERT);
    }

    public void setDepoOutFlat(){
        wrist.setPosition(R_OUT_FLAT);
        arm.setPosition(L_OUT_FLAT);
    }

    public void setDepoIn(){
        wrist.setPosition(RIGHT_IN);
        arm.setPosition(LEFT_IN);
    }*/

}

