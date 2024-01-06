package overcharged.components;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class hang {
    public OcServo hang;
    public static float IN = 170f;
    public static float HANG = 100f;

    public hang(HardwareMap hardwareMap, boolean isRight) {
        if(isRight){
            IN = 1f;//29f;
            HANG = 126f;
            hang = new OcServo(hardwareMap, "rightHang", IN);
        } else {
            IN = 208f;
            HANG = 95f;
            hang = new OcServo(hardwareMap, "leftHang", IN);
        }
    }
    public void setPosition(float pos){
        hang.setPosition(pos);
    }

    public void setIn() {
        hang.setPosition(IN);
    }

    public void setHang() {
        hang.setPosition(HANG);
    }
}
