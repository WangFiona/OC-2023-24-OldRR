package overcharged.components;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class droneShooter {
    public OcServo droneShooter;
    public static final float INIT = 55f;
    public static float SHOOT = 166f;

    public droneShooter(HardwareMap hardwareMap) {
        droneShooter = new OcServo(hardwareMap, "droneShooter", INIT);
    }
    public void setPosition(float pos){
        droneShooter.setPosition(pos);
    }

    public void setInit() { droneShooter.setPosition(INIT); }

    public void setShoot() { droneShooter.setPosition(SHOOT); }
}