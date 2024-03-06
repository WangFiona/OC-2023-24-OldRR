package overcharged.components;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class newDepo {

    public OcServo frontClaw;
    public OcServo backClaw;
    public OcServo depoL;
    public OcServo depoR;
    public AnalogInput armVolt;

    public static float FRONT_CLOSE = 31f;
    public static float FRONT_DUMP = 151f;
    //public static float FRONT_OPEN = 151f;
    public static float BACK_CLOSE = 216f;
    public static float BACK_DUMP = 98f;
    //public static float BACK_OPEN = 98f;

    public static float WRIST_IN_VERT = 146f;//53f;
    public static float WRIST_OPP_VERT = 10f;//237f;
    public static float WRIST_OPP_FLAT = 57f;//142f;
    public static float WRIST_FLAT = 237f;//142f;
    public static float WRIST_R_DIAG= 117f;//107f;
    public static float WRIST_L_DIAG = 181f;//88f;

    public static float LEFT_IN = 127f;
    public static float RIGHT_IN = 18f;

    public static float L_VERT = 47f;
    public static float R_VERT = 106f;
    public static float L_FLAT = 118f;
    public static float R_FLAT = 177f;
    public static float L_P_DIAG = 75f;
    public static float R_P_DIAG = 134f;
    public static float L_N_DIAG = 166f;
    public static float R_N_DIAG = 225f;
    public static float L_OPP_VERT = 192f;
    public static float R_OPP_VERT = 251f;

    public static float ARM_IN = 174f;
    public static float ARM_OUT = 70f;

    public newDepo(HardwareMap hardwareMap) {
        frontClaw = new OcServo(hardwareMap, "frontClaw", FRONT_DUMP);
        backClaw = new OcServo(hardwareMap, "backClaw", BACK_DUMP);
        depoL = new OcServo(hardwareMap, "depoL", LEFT_IN);
        depoR = new OcServo(hardwareMap, "depoR", RIGHT_IN);
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
        depoL.setPosition(pos);
    }

    public void setArmPos(float pos){
        depoR.setPosition(pos);
    }

    public double getArmVolt(){
        return armVolt.getVoltage(); //0 to 3.3
    }

    public void setBothDepoPosition(int rPos, int lPos){
        depoR.setPosition(rPos);
        depoL.setPosition(lPos);
    }

    public void setDepoOutVert(){
        depoR.setPosition(R_VERT);
        depoL.setPosition(L_VERT);
    }
    public void setDepoOutOppVert(){
        depoR.setPosition(R_OPP_VERT);
        depoL.setPosition(L_OPP_VERT);
    }

    public void setDepoOutPDiag(){
        depoR.setPosition(R_P_DIAG);
        depoL.setPosition(L_P_DIAG);
    }

    public void setDepoOutNDiag(){
        depoR.setPosition(R_N_DIAG);
        depoL.setPosition(L_N_DIAG);
    }

    public void setDepoOutFlat(){
        depoR.setPosition(R_FLAT);
        depoL.setPosition(L_FLAT);
    }

    public void setDepoIn(){
        depoR.setPosition(RIGHT_IN);
        depoL.setPosition(LEFT_IN);
    }

}

