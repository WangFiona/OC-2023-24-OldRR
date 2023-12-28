package overcharged.components;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class pixelDropper {
    public OcServo pixelDropper;
    public static float IN = 100f;
    public static float DUMP = 150f;

    public pixelDropper(HardwareMap hardwareMap, boolean isRight) {
        if(isRight){
            IN = 100f;
            DUMP = 146f;
            pixelDropper = new OcServo(hardwareMap, "rightPixel", 100f);
        } else {
            IN = 156f;
            DUMP = 112f;
            pixelDropper = new OcServo(hardwareMap, "leftPixel", 156f);
        }
    }
    public void setPosition(float pos){
        pixelDropper.setPosition(pos);
    }

    public void setIn() { pixelDropper.setPosition(IN); }

    public void setDump() { pixelDropper.setPosition(DUMP); }
}
