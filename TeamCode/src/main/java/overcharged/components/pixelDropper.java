package overcharged.components;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class pixelDropper {
    public OcServo leftPixelDropper;
    public OcServo rightPixelDropper;
    public static float LEFT_IN = 169f;
    public static float LEFT_DUMP = 111f;
    public static float RIGHT_IN = 88f;
    public static float RIGHT_DUMP = 148f;

    public pixelDropper(HardwareMap hardwareMap, boolean isRight) {
        leftPixelDropper = new OcServo(hardwareMap, "leftPixel", LEFT_IN);
        rightPixelDropper = new OcServo(hardwareMap, "rightPixel", RIGHT_IN);
    }
    public void setLeftPosition(float pos){
        leftPixelDropper.setPosition(pos);
    }

    public void setLeftIn() { leftPixelDropper.setPosition(LEFT_IN); }

    public void setLeftDump() { leftPixelDropper.setPosition(LEFT_DUMP); }

    public void setRightPosition(float pos){
        rightPixelDropper.setPosition(pos);
    }

    public void setRightIn() { rightPixelDropper.setPosition(RIGHT_IN); }

    public void setRightDump() { rightPixelDropper.setPosition(RIGHT_DUMP); }
}
