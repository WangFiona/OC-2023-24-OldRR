package overcharged.components;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class intakeSmallTilt {
    public OcServo intakeSmallTilt;
    public static final float INIT = 174f;
    public static final float TRANSFER = 174f;//115f;
    public static final float FLAT = 183f;
    public static final float OUT = 105f;
    public static final float DUMP_EXTRA = 190f;

    //for auto intaking, will need SEVERE tuning
    int pixelInterval = 4;
    public final float FIFTHP = OUT-pixelInterval*4;
    public final float FOURTHP = OUT-pixelInterval*3;
    public final float THIRDP = OUT-pixelInterval*2;
    public final float SECONDP = OUT-pixelInterval;

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
