package overcharged.components;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class intakeBigTilt {
    public OcServo intakeBigTilt;
    public static final float INIT = 204f;//230f;
    public static final float TRANSFER = 204f;
    public static final float FLAT = 189f;
    public static final float OUT = 83f;

    public intakeBigTilt(HardwareMap hardwareMap) {
        intakeBigTilt = new OcServo(hardwareMap, "intakeBigTilt", TRANSFER);
    }
    public void setPosition(float pos){
        intakeBigTilt.setPosition(pos);
    }

    public void setInit() { intakeBigTilt.setPosition(INIT); }

    public void setTransfer() { intakeBigTilt.setPosition(TRANSFER); }

    public void setFlat() { intakeBigTilt.setPosition(FLAT); }

    public void setOut() { intakeBigTilt.setPosition(OUT); }
}
