package overcharged.components;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class hang {
    public OcServo hang;
    public static float IN = 248f;
    public static float HANG = 59f;

    public hang(HardwareMap hardwareMap, boolean isRight) {
        if(isRight){
            IN = 248f;//29f;
            HANG = 59f;
            hang = new OcServo(hardwareMap, "rightHang", IN);
        } else {
            IN = 19f;//217f;
            HANG = 217f;
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
