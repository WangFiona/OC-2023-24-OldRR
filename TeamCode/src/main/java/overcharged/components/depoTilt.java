package overcharged.components;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class depoTilt {
    public OcServo depoTilt;
    public static final float IN = 155f;
    public static float OUT = 45f;

    public depoTilt(HardwareMap hardwareMap) {
        depoTilt = new OcServo(hardwareMap, "depoTilt", IN);
    }
    public void setPosition(float pos){
        depoTilt.setPosition(pos);
    }

    public void setIn() { depoTilt.setPosition(IN); }

    public void setOut() { depoTilt.setPosition(OUT); }
}
