package overcharged.components;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class pixelDropper {
    public OcServo pixelDropper;
    public static float IN = 127f;
    public static float DUMP = 192f;

    public pixelDropper(HardwareMap hardwareMap, boolean isRight) {
        if(isRight){
            IN = 127f;
            DUMP = 192f;
            pixelDropper = new OcServo(hardwareMap, "rightPixel", 100f);
        } else {
            IN = 123f;
            DUMP = 68f;
            pixelDropper = new OcServo(hardwareMap, "leftPixel", 156f);
        }
    }
    public void setPosition(float pos){
        pixelDropper.setPosition(pos);
    }

    public void setIn() { pixelDropper.setPosition(IN); }

    public void setDump() { pixelDropper.setPosition(DUMP); }
}
