package Common;

/** 
 * Creates a new Limelight Pipeline using the pipelines ID and LED state
 * 
*/
public class LimelightPipeline {
    private double id;
    private boolean ledState;
  
    
    /**
     * Create Limelight piepline object
     * @param id
     * @param ledState True = On
     */
    public LimelightPipeline(double id, boolean ledState) {
      this.id = id;
      this.ledState = ledState;
    }

    /**
     * 
     * @return Get ID of pipeline
     */
    public double getId() {
      return id;
    }

    /**
     * 
     * @return Get LED state as boolean value <code>True | False</code>
     */
    public boolean getLedState() {
      return ledState;
    }

}