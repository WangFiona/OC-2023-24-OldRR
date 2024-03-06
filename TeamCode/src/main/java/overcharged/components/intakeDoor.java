package overcharged.components;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class intakeDoor {
    public OcServo intakeDoor;
    public static final float CLOSED = 64f;
    public static float OPEN = 205f;

    public intakeDoor(HardwareMap hardwareMap) {
        intakeDoor = new OcServo(hardwareMap, "intakeDoor", CLOSED);
    }
    public void setPosition(float pos){
        intakeDoor.setPosition(pos);
    }

    public void setClosed() { intakeDoor.setPosition(CLOSED); }

    public void setOpen() { intakeDoor.setPosition(OPEN); }
}
