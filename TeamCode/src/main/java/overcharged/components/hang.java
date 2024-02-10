package overcharged.components;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class hang {
    public OcServo rightHang;
    public OcServo leftHang;
    public static float RIGHT_IN = 162f;
    public static float RIGHT_HANG = 208f;
    public static float LEFT_IN = 36f;
    public static float LEFT_HANG = 79f;

    public hang(HardwareMap hardwareMap, boolean isRight) {
        rightHang = new OcServo(hardwareMap, "rightHang", RIGHT_IN);
        leftHang = new OcServo(hardwareMap, "leftHang", LEFT_IN);
    }
    public void setLeftPosition(float pos){
        leftHang.setPosition(pos);
    }

    public void setLeftIn() {
        leftHang.setPosition(LEFT_IN);
    }

    public void setLeftHang() {
        leftHang.setPosition(LEFT_HANG);
    }

    public void setRightPosition(float pos){
        rightHang.setPosition(pos);
    }

    public void setRightIn() {
        rightHang.setPosition(RIGHT_IN);
    }

    public void setRightHang() {
        rightHang.setPosition(RIGHT_HANG);
    }
}
