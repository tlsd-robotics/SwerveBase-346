package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

public class Superstructure {
    private final PneumaticHub pneumatics = new PneumaticHub(Constants.Superstructure.PNEUMATICS_HUB_ID);
    private final  PowerDistribution pdp = new PowerDistribution(Constants.Superstructure.POWER_DISTRIBUTION_ID, ModuleType.kRev);

    public Superstructure() {
        pneumatics.enableCompressorDigital();
        pdp.clearStickyFaults();

        CameraServer.startAutomaticCapture("USB_CAM0", 0);
    }
}
