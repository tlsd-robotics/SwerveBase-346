package Common;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.I2C.Port;

public class NavX {

    // Create NavX object that will be used by anything that needs it. 
    // creating it here rather than a subsystem will ensure that it will always be accessable regardless of which subystems 
    // are busy running commands and whatnot
    public static AHRS gyroscope = new AHRS(Port.kMXP, (byte) 200);

    /**
     * @return Returns the created NavX object
     */
    public static AHRS getNavX() {
        return gyroscope;
    }
    /**
     * Zeroes the yaw of the NavX object
     */
    public static void zeroYaw() {
        gyroscope.zeroYaw();
    }

    /**
     * Zeroes the displacement of the NavX
     */
    public static void zeroDisplacement() {
        gyroscope.resetDisplacement();
    }

}
