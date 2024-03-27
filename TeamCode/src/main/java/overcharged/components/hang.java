package overcharged.components;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class hang {
    public OcServo Hang;
    public static float IN = 162f;
    public static float HANG = 208f;

    public hang(HardwareMap hardwareMap, boolean isRight) {
        Hang = new OcServo(hardwareMap, "Hang", IN);
    }
    public void setPosition(float pos){Hang.setPosition(pos);
    }

    public void setIn() {
        Hang.setPosition(IN);
    }

    public void setHang() {
        Hang.setPosition(HANG);
    }

}
