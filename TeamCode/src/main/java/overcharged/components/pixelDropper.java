package overcharged.components;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class pixelDropper {
    public OcServo pixelDropper;
    public static float IN = 100f;
    public static float DUMP = 100f;

    public pixelDropper(HardwareMap hardwareMap, boolean isRight) {
        if(isRight){
            IN = 86f;
            DUMP = 148f;
            pixelDropper = new OcServo(hardwareMap, "rightPixel", IN);
        } else {
            IN = 170f;
            DUMP = 110f;
            pixelDropper = new OcServo(hardwareMap, "leftPixel", IN);
        }
    }
    public void setPosition(float pos){
        pixelDropper.setPosition(pos);
    }

    public void setIn() { pixelDropper.setPosition(IN); }

    public void setDump() { pixelDropper.setPosition(DUMP); }
}
