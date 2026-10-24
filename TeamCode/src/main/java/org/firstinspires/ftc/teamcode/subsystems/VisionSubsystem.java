package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

/*
 * Vision Subsystem
 * Owns the LIMELIGHT 3A. It only READS the camera. It never moves the turret. Another class
 * (for now VisionAlpha) takes the answers from here and tells the turret what to do.
 *
 * WHAT IT ANSWERS: "Where is OUR hive's AprilTag cluster, as seen from the turret?"
 *   getTxDegrees()  -> how far LEFT or RIGHT of the camera's center the target is.
 *                      POSITIVE means the target is to the RIGHT (that is the Limelight's rule).
 *                      Careful: the turret's angle is positive to the LEFT, so the two signs are
 *                      opposite. The OpMode that aims the turret has to flip the sign.
 *   getTyDegrees()  -> how far UP or DOWN. Positive is up. Distance will come from this later.
 *
 * HOW IT PICKS THE TARGET (each update):
 *   1. Throw away every tag that is not on OUR alliance's hive (see setAlliance()).
 *   2. Both of our cells can be in view at once. The RAISED cell is the higher one, and the
 *      higher one is the one with the larger ty, so we pick the cell holding the highest tag.
 *   3. Average tx and ty over all the visible tags in that ONE cell. Each cell has four tags
 *      spread about 13 inches across, so aiming at a single tag would aim up to 6.5 inches off
 *      center. The average lands near the middle of the cluster, which is the middle of the cell
 *      opening side to side. (The arm can hide some tags, so we use whatever is visible.)
 *   [CONFIRM] If the arm hides only the LEFT pair or only the RIGHT pair, the average shifts
 *   toward the visible side. Look at this on the real hive and tell us how big the error is.
 *
 * STATE (what we last saw) is NO_TARGET or TARGET_VISIBLE.
 * STATUS (what the camera says) is the numbers above, plus getResultAgeMs() and isCameraConnected().
 *
 * The OpMode must call update() ONCE per loop(). Nothing calls it for us.
 *
 * Robot configuration (on the Driver Station):
 *   Ethernet Device name: limelight   (Limelight 3A, USB-C to USB-A into the Control Hub's blue
 *                                      USB 3.0 port. The Control Hub sees it as an Ethernet device.)
 * The Limelight needs about 30 seconds to boot after power-up, so wait for it before testing.
 * In the Limelight's web page (http://172.29.0.1:5801 from a laptop on the Control Hub's Wi-Fi),
 * pipeline APRILTAG_PIPELINE_INDEX must be an AprilTag (Fiducial) pipeline for the 36h11 family.
 */
public class VisionSubsystem {

    // ------------------------------------------------------------------
    // Hardware name. This MUST match the robot configuration exactly.
    // ------------------------------------------------------------------
    private static final String LIMELIGHT_NAME = "limelight";

    // ------------------------------------------------------------------
    // Camera settings
    // ------------------------------------------------------------------
    // Which pipeline (0 to 9 in the Limelight's web page) finds the AprilTags.
    // [CONFIRM] Change this if your AprilTag pipeline is not number 0.
    private static final int APRILTAG_PIPELINE_INDEX = 0;

    // A camera result older than this many milliseconds is too old to aim with, so we treat
    // it as "no target". The Limelight runs at 90 FPS or so, which is about 11 ms per frame,
    // so a result this old means the camera stopped sending.
    private static final long MAX_RESULT_AGE_MS = 200;

    // ------------------------------------------------------------------
    // Alliance and tag numbers (from the Competition Manual, see biobuzz-hive-apriltag-geometry.md)
    // ------------------------------------------------------------------
    // The tags run 30 to 45, in groups of four. Each group of four is ONE cell's sticker:
    //   30-33 red far side      34-37 red audience side
    //   38-41 blue audience     42-45 blue far side
    // Counting the cells from 30, that is cell 0 and 1 for RED, and cell 2 and 3 for BLUE.
    private static final int FIRST_TAG_ID = 30;
    private static final int LAST_TAG_ID = 45;
    private static final int TAGS_PER_CELL = 4;
    private static final int CELLS_PER_ALLIANCE = 2;
    private static final int RED_FIRST_CELL = 0;
    private static final int BLUE_FIRST_CELL = 2;

    // This means "no cell". Real cells are numbered 0 to 3.
    private static final int NO_CELL = -1;

    // The alliance we start with. An OpMode can change it any time with setAlliance().
    private static final Alliance DEFAULT_ALLIANCE = Alliance.BLUE;

    // ------------------------------------------------------------------
    // State
    // ------------------------------------------------------------------
    public enum Alliance {
        RED,
        BLUE
    }

    // STATE = what we last saw.
    public enum VisionState {
        NO_TARGET,       // No fresh, valid result with one of OUR tags in it.
        TARGET_VISIBLE   // At least one of OUR tags is in view.
    }

    private final Limelight3A limelight;

    private Alliance alliance = DEFAULT_ALLIANCE;
    private VisionState state = VisionState.NO_TARGET;

    // The newest answers. They are zero when there is no target.
    private double txDegrees = 0.0;
    private double tyDegrees = 0.0;
    private int tagCount = 0;
    private int targetCell = NO_CELL;
    private long resultAgeMs = -1;

    // The Limelight may still be booting when we are constructed, and then the pipeline switch
    // does not work. So update() keeps trying until it works once.
    private boolean pipelineSet = false;

    // ==================================================================
    // SETUP
    // ==================================================================

    public VisionSubsystem(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);

        // Try to pick the AprilTag pipeline now. If the camera is not ready, update() retries.
        pipelineSet = limelight.pipelineSwitch(APRILTAG_PIPELINE_INDEX);

        // Starts polling for data. Without start(), getLatestResult() never has anything.
        limelight.start();
    }

    // ==================================================================
    // ACTIONS: the OpMode tells the camera what to do
    // ==================================================================

    /*
     * Choose which hive's tags to look for. RED looks for tags 30-37, BLUE for tags 38-45.
     */
    public void setAlliance(Alliance newAlliance) {
        alliance = newAlliance;
    }

    /*
     * Stop polling the camera. Call this from the OpMode's stop().
     */
    public void stop() {
        limelight.stop();
    }

    /*
     * Call this ONCE per loop(). It reads the newest camera result and picks the target.
     */
    public void update() {
        // The camera may have finished booting since the last loop.
        if (!pipelineSet && limelight.isConnected()) {
            pipelineSet = limelight.pipelineSwitch(APRILTAG_PIPELINE_INDEX);
        }

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            resultAgeMs = -1;
            clearTarget();
            return;
        }

        resultAgeMs = result.getStaleness();
        if (resultAgeMs > MAX_RESULT_AGE_MS) {
            clearTarget();
            return;
        }

        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();

        // Pass 1: find the cell that holds the HIGHEST tag (largest ty) among OUR tags.
        int bestCell = NO_CELL;
        double highestTy = -Double.MAX_VALUE;
        for (LLResultTypes.FiducialResult tag : tags) {
            int cell = getCellNumber(tag.getFiducialId());
            if (!isOurCell(cell)) {
                continue;
            }
            if (tag.getTargetYDegrees() > highestTy) {
                highestTy = tag.getTargetYDegrees();
                bestCell = cell;
            }
        }

        if (bestCell == NO_CELL) {
            clearTarget();
            return;
        }

        // Pass 2: average tx and ty over the visible tags in that one cell.
        double txTotal = 0.0;
        double tyTotal = 0.0;
        int tagsInCell = 0;
        for (LLResultTypes.FiducialResult tag : tags) {
            if (getCellNumber(tag.getFiducialId()) == bestCell) {
                txTotal = txTotal + tag.getTargetXDegrees();
                tyTotal = tyTotal + tag.getTargetYDegrees();
                tagsInCell = tagsInCell + 1;
            }
        }

        txDegrees = txTotal / tagsInCell;
        tyDegrees = tyTotal / tagsInCell;
        tagCount = tagsInCell;
        targetCell = bestCell;
        state = VisionState.TARGET_VISIBLE;
    }

    // ==================================================================
    // QUESTIONS: the OpMode asks what the camera sees
    // ==================================================================

    public boolean hasTarget() {
        return state == VisionState.TARGET_VISIBLE;
    }

    /*
     * Degrees LEFT or RIGHT of the camera's center. POSITIVE = target is to the RIGHT.
     * Zero when there is no target, so check hasTarget() first.
     */
    public double getTxDegrees() {
        return txDegrees;
    }

    /*
     * Degrees UP or DOWN. Positive = up. Zero when there is no target.
     */
    public double getTyDegrees() {
        return tyDegrees;
    }

    /*
     * How many tags of the target cell we are using right now (1 to 4).
     */
    public int getTagCount() {
        return tagCount;
    }

    /*
     * Which cell we are aiming at: 0 or 1 for RED, 2 or 3 for BLUE (counting up from tag 30,
     * four tags to a cell). NO_CELL (-1) when there is no target.
     */
    public int getTargetCell() {
        return targetCell;
    }

    public Alliance getAlliance() {
        return alliance;
    }

    public VisionState getState() {
        return state;
    }

    /*
     * How old the newest camera result is, in milliseconds. -1 means no valid result yet.
     */
    public long getResultAgeMs() {
        return resultAgeMs;
    }

    public boolean isCameraConnected() {
        return limelight.isConnected();
    }

    /*
     * Shows the camera's health and the target. Call it after update(). It prints its own
     * heading, so the OpMode has nothing else to call.
     */
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addLine("--- VISION ---");
        telemetry.addData("Alliance", alliance);

        if (!limelight.isConnected()) {
            telemetry.addData("Limelight", "NOT CONNECTED (check the USB cable and the config name)");
        } else {
            LLStatus status = limelight.getStatus();
            if (status != null) {
                telemetry.addData("Limelight", "Temp %.1fC   CPU %.0f%%   FPS %d",
                        status.getTemp(), status.getCpu(), (int) status.getFps());
                telemetry.addData("Pipeline", "%d (wanted %d)",
                        status.getPipelineIndex(), APRILTAG_PIPELINE_INDEX);
            } else {
                telemetry.addData("Limelight", "connected, no status yet");
            }
        }

        telemetry.addData("Vision State", state);
        if (state == VisionState.TARGET_VISIBLE) {
            telemetry.addData("Target cell", "%d   (%d tags used)", targetCell, tagCount);
            telemetry.addData("tx", "%.1f deg  (+ = target is to the RIGHT)", txDegrees);
            telemetry.addData("ty", "%.1f deg  (+ = up)", tyDegrees);
        }
        telemetry.addData("Result age", "%d ms", resultAgeMs);
    }

    // ==================================================================
    // PRIVATE HELPERS
    // ==================================================================

    private void clearTarget() {
        state = VisionState.NO_TARGET;
        txDegrees = 0.0;
        tyDegrees = 0.0;
        tagCount = 0;
        targetCell = NO_CELL;
    }

    /*
     * Which cell a tag belongs to, counting from tag 30 in groups of four.
     * Returns NO_CELL for a tag that is not a BIOBUZZ hive tag.
     */
    private int getCellNumber(int tagId) {
        if (tagId < FIRST_TAG_ID || tagId > LAST_TAG_ID) {
            return NO_CELL;
        }
        return (tagId - FIRST_TAG_ID) / TAGS_PER_CELL;
    }

    /*
     * True when the cell belongs to OUR alliance's hive.
     */
    private boolean isOurCell(int cell) {
        int firstCell = RED_FIRST_CELL;
        if (alliance == Alliance.BLUE) {
            firstCell = BLUE_FIRST_CELL;
        }
        return cell >= firstCell && cell < firstCell + CELLS_PER_ALLIANCE;
    }
}
