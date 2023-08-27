package frc.robot;

import Common.Limelight;
import Common.LimelightPipeline;
import Common.Target;
import Common.Target.DetectionType;

public class Vision {

  // =======================================================================================================
  // ================================= Timelight Pipelines and Getters =====================================
  public static class Pipelines {

    // Example Pipelines. These are the main three you need. However if you require more targeting options, just link the id to the network table id found in the limelight configuring tool
    private static LimelightPipeline defaultPL = new LimelightPipeline(0, false);
    private static LimelightPipeline reflectivePL = new LimelightPipeline(1, true);
    private static LimelightPipeline apriltagPL = new LimelightPipeline(2, false);
    private static LimelightPipeline gamepiecePL = new LimelightPipeline(3, false);

    /**
     * @return The default Pipeline of the limelight. This can be anything, but I recommend a full color vision and no led so it can serve as a drive camera when not aiming.
     */
    public static LimelightPipeline getDefaultPipeline() {
      return defaultPL;
    }

    /**
     * @return The Reflective tape targeting pipeline. 
     */
    public static LimelightPipeline getReflectivePipeline() {
      return reflectivePL;
    }

    /**
     * @return The Apriltag targeting pipeline
     */
    public static LimelightPipeline getApriltagPipeline() {
      return apriltagPL;
    }

    /**
     * @return The Gamepiece targeting pipeline
     */
    public static LimelightPipeline getGamepicePL() {
      return gamepiecePL;
    }
  }

  // ======================================================================
  // ============================ Limelights =================================

  public static class Limelights {
    public static Limelight defaultLimelight = new Limelight("limelight", Pipelines.getDefaultPipeline(), 15, 15);

    public static Limelight getDefaultLimelight() {
      return defaultLimelight;
    }
  }

  
  // ======================================================================
  // ============================ Targets =================================

  public static class Targets {
    public static Target exampleTarget = new Target("example", 36.0, DetectionType.Reflective, 0.5);
  }


  /**
   * Sets the provided limelight's pipeline to the specified pipeline.
   * @param limelight Limelight Object
   * @param pipeline Piepline Object
   */
  public static void setPipeline(Limelight limelight, LimelightPipeline pipeline) {
    limelight.setPipeline(pipeline);
  }
}
