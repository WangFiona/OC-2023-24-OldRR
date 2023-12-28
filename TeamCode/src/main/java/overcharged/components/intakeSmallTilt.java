package overcharged.components;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class intakeSmallTilt {
    public OcServo intakeSmallTilt;
    public static final float INIT = 230f;
    public static final float TRANSFER = 66f;//115f;
    public static final float FLAT = 99f;
    public static final float OUT = 5f;

    public intakeSmallTilt(HardwareMap hardwareMap) {
        intakeSmallTilt = new OcServo(hardwareMap, "intakeSmallTilt", TRANSFER);
    }
    public void setPosition(float pos){
        intakeSmallTilt.setPosition(pos);
    }

    public void setInit() { intakeSmallTilt.setPosition(INIT); }

    public void setTransfer() { intakeSmallTilt.setPosition(TRANSFER); }

    public void setFlat() { intakeSmallTilt.setPosition(FLAT); }

    public void setOut() { intakeSmallTilt.setPosition(OUT); }
}
