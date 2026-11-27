package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

/*
 * Vision Subsystem
 * Owns the LIMELIGHT 3A. It only READS the camera. It never moves the turret. Another class
 * takes the answers from here and tells the turret what to do.
 *
 * WHAT IT ANSWERS: "Where is OUR hive's AprilTag cluster, as seen from the turret?"
 *   getTxDegrees()  -> how far LEFT or RIGHT of the camera's center the target is, PLUS the
 *                      AIM_OFFSET_DEGREES correction (zero by default).
 *                      POSITIVE means the target is to the RIGHT (that is the Limelight's rule).
 *                      Careful: the turret's angle is positive to the LEFT, so the two signs are
 *                      opposite. The OpMode that aims the turret has to flip the sign.
 *   getTyDegrees()  -> how far UP or DOWN. Positive is up. Distance will come from this later.
 *
 * HOW IT PICKS THE TARGET (each update):
 *   1. Throw away every tag that is not on OUR alliance's hive (see setAlliance()).
 *   2. Both of our cells can be in view at once. The RAISED cell is the higher one, and the
 *      higher one is the one with the larger ty, so we pick the cell holding the highest tag.
 *      THE LOCK: once we pick a cell we STAY on it, and only switch to our other cell when that
 *      one is higher by CELL_SWITCH_MARGIN_DEGREES. Without this the target would flip back and
 *      forth while the hive tips, when both cells are about the same height. If the cell we are
 *      locked on drops out of view, we wait LOCK_RELEASE_SECONDS (reporting NO_TARGET, so the
 *      turret holds) before we accept the other cell. A quick dropout, like the arm passing in
 *      front of the tags, then does not throw the lock away. After a hive tip the robot drives
 *      to the other side, so the old cell stays out of view, the lock releases, and the cell now
 *      in view becomes the new target.
 *      ONE LIMIT: if only the LOWERED cell is in view, ty alone cannot say it is lowered (that
 *      needs a distance), so we would still aim at it.
 *   3. Average tx and ty over all the visible tags in that ONE cell. Each cell has four tags
 *      spread about 13 inches across, so aiming at a single tag would aim up to 6.5 inches off
 *      center. The average lands near the middle of the cluster, which is the middle of the cell
 *      opening side to side. (The arm can hide some tags, so we use whatever is visible.)
 
 * THE AUDIENCE CELL STATUS (getAudienceCellStatus(), issue #134):
 *   Is the AUDIENCE-side cell of OUR hive raised (UP) or lowered (DOWN)?
 *   The tag NUMBERS cannot tell us: tags 34-37 are the red audience cell whether it is up or
 *   down. HEIGHT can. The raised cell's tags are higher than the lowered cell's tags (about
 *   50 in versus 34 in), so when BOTH of our cells are in view we compare them:
 *     audience cell higher by CELL_SWITCH_MARGIN_DEGREES or more  -> UP
 *     audience cell lower by CELL_SWITCH_MARGIN_DEGREES or more   -> DOWN
 *     anything else                                               -> UNKNOWN
 *   UNKNOWN is a real answer, not an error. It means only one of our cells is in view (ty alone
 *   cannot say how high a cell is, because ty also depends on distance), or the two cells look
 *   about the same height (the hive is in the middle of a tip).
 *   WHERE YOU STAND MATTERS: the tags face the side of the RAISED cell. From the audience side
 *   with the audience cell DOWN, the tags may face away from the camera, and then we see nothing
 *   at all. That is UNKNOWN, not DOWN. To see DOWN you will probably need to stand on the FAR
 *   side of the hive. This only REPORTS. It does not change which cell we aim at.
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
    private static final int APRILTAG_PIPELINE_INDEX = 0;

    // How often (in seconds) update() asks the camera "which pipeline are you on?". If it is not
    // the AprilTag one, update() tells it to switch back. WHY: the Limelight can restart in the
    // middle of a match (for example when a low battery makes the Control Hub's USB port dip).
    // It then starts on whatever pipeline it boots into, and the pipeline we picked at INIT is
    // gone. We used to pick the pipeline only ONCE, so after a restart nothing switched it back
    // and the camera never saw the hive again. Smaller = notices sooner, but asks the camera
    // more often.
    private static final double PIPELINE_CHECK_SECONDS = 0.5;

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

    // How many cells there are in all (4, so cells 0 to 3).
    private static final int TOTAL_CELLS = (LAST_TAG_ID - FIRST_TAG_ID + 1) / TAGS_PER_CELL;

    // Which cells are on the FAR side of the field (the manual calls it the far, scoring side).
    // The other cell of each hive is on the AUDIENCE side. Red: cell 0 is far, cell 1 is audience.
    // Blue: cell 2 is audience, cell 3 is far. (Tags 30-33 red far, 34-37 red audience,
    // 38-41 blue audience, 42-45 blue far.)
    private static final int RED_FAR_CELL = 0;
    private static final int BLUE_FAR_CELL = 3;

    // The AUDIENCE cell of each hive: the other cell of the same hive. Red: cell 1 (tags 34-37).
    // Blue: cell 2 (tags 38-41). Used by the audience cell status (issue #134).
    private static final int RED_AUDIENCE_CELL = 1;
    private static final int BLUE_AUDIENCE_CELL = 2;

    // This means "no cell". Real cells are numbered 0 to 3.
    private static final int NO_CELL = -1;

    // ------------------------------------------------------------------
    // Aim offset (the fudge factor)
    // ------------------------------------------------------------------
    // A steady correction, in degrees, added to the camera's tx. Use it when the shots land
    // to one side even though the turret says it is aimed. Two things cause that: the camera is
    // not on the shooter's centerline, and the middle of the tag cluster is not exactly the
    // middle of the cell opening.
    //   POSITIVE -> the turret aims a little more to the RIGHT of where the camera is centered.
    //   NEGATIVE -> the turret aims a little more to the LEFT.
    // Start at 0.0. Shoot from one fixed spot. If the shots land to the right of the hole, make
    // this number negative. If they land left, make it positive. Change it a little at a time
    // (0.5 degrees), because 1 degree is about 1 inch of sideways miss for every 5 feet of range.
    // It is only applied when there is a target.
    private static final double AIM_OFFSET_DEGREES = 0.0;

    // ------------------------------------------------------------------
    // The lock on one cell (see "THE LOCK" at the top)
    // ------------------------------------------------------------------
    // Our other cell has to be higher than the locked cell by this many degrees of ty before we
    // switch to it. Why 3: with the camera 10 inches off the floor, the raised and lowered
    // clusters are about 7 degrees apart at 10 feet and more than 12 degrees apart at 6 feet, so a
    // real switch is far bigger than 3, while noise in ty is only a degree or so.
    // Raise it if the target flips during a tip. Lower it if it is slow to switch.
    private static final double CELL_SWITCH_MARGIN_DEGREES = 3.0;

    // If the locked cell is out of view, wait this many seconds before we accept our other cell.
    // Long enough to ride out the arm hiding the tags, short enough to be ready
    // quickly once the robot has driven to the other side of the hive.
    private static final double LOCK_RELEASE_SECONDS = 0.5;

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

    // Is our AUDIENCE-side cell raised or lowered? See "THE AUDIENCE CELL STATUS" at the top.
    // TODO (issue #134): test on the real blue hive. Check that UP and DOWN come out right when
    // the hive is flipped, and that the 3 degree margin (CELL_SWITCH_MARGIN_DEGREES) is not so
    // big that it says UNKNOWN too often. Test from the audience side and from the far side.
    public enum CellStatus {
        UP,       // Both of our cells are in view and the audience cell is the higher one.
        DOWN,     // Both of our cells are in view and the audience cell is the lower one.
        UNKNOWN   // We cannot tell: only one cell in view, or the two look the same height.
    }

    private final Limelight3A limelight;

    private Alliance alliance = DEFAULT_ALLIANCE;
    private VisionState state = VisionState.NO_TARGET;
    private CellStatus audienceCellStatus = CellStatus.UNKNOWN;

    // The newest answers. They are zero when there is no target.
    // txDegrees is the camera's tx PLUS AIM_OFFSET_DEGREES. rawTxDegrees is the camera's own tx.
    private double txDegrees = 0.0;
    private double rawTxDegrees = 0.0;
    private double tyDegrees = 0.0;
    private int tagCount = 0;
    private int targetCell = NO_CELL;
    private long resultAgeMs = -1;

    // The cell we are locked onto, or NO_CELL. lockedCellUnseenTimer counts the seconds since we
    // last saw the locked cell.
    private int lockedCell = NO_CELL;
    private final ElapsedTime lockedCellUnseenTimer = new ElapsedTime();

    // For each cell: is any of its tags in view this update, and how high is its highest tag
    // (the largest ty)? They are filled in by every update(), so we do not make new ones each loop.
    private final boolean[] cellVisible = new boolean[TOTAL_CELLS];
    private final double[] highestTyInCell = new double[TOTAL_CELLS];

    // The Limelight may still be booting when we are constructed, and then the pipeline switch
    // does not work. So update() keeps trying until it works. It is also set back to false
    // whenever the camera drops off (see update()), so the pipeline is picked again after a
    // restart.
    private boolean pipelineSet = false;

    // Counts the seconds since update() last asked the camera which pipeline it is on.
    private final ElapsedTime pipelineCheckTimer = new ElapsedTime();

    // How many times the camera needed its pipeline fixed during the run: it dropped off after we
    // had picked the pipeline (so it has to be picked again), or update() found it on the WRONG
    // pipeline and switched it back. Shown on the screen: a number above 0 means the camera
    // restarted (or lost its settings) during the run.
    private int pipelineFixCount = 0;

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
        // The cell we were locked on belongs to the OTHER hive now, so let go of it.
        lockedCell = NO_CELL;
    }

    /*
     * Stop polling the camera. Call this from the OpMode's stop().
     */
    public void stop() {
        limelight.stop();
    }

    /*
     * Makes sure the camera is on the AprilTag pipeline. update() calls it every loop.
     *   1. Camera not connected (booting, unplugged, or it just restarted): forget that we picked
     *      the pipeline. We cannot talk to it yet, so the next step waits for it to come back.
     *   2. Camera connected but we have not picked the pipeline yet (or it just came back): pick it.
     *   3. Camera connected and we picked it: every PIPELINE_CHECK_SECONDS, ask which pipeline it
     *      is really on. If it is not ours (it restarted so fast that it never looked
     *      disconnected), switch it back.
     */
    private void checkPipeline() {
        boolean connected = limelight.isConnected();

        if (!connected) {
            if (pipelineSet) {
                // It WAS working and just dropped off: it will need the pipeline picked again.
                pipelineFixCount = pipelineFixCount + 1;
            }
            pipelineSet = false;
            return;
        }

        if (!pipelineSet) {
            pipelineSet = limelight.pipelineSwitch(APRILTAG_PIPELINE_INDEX);
            pipelineCheckTimer.reset();
            return;
        }

        if (pipelineCheckTimer.seconds() < PIPELINE_CHECK_SECONDS) {
            return;
        }
        pipelineCheckTimer.reset();

        LLStatus status = limelight.getStatus();
        if (status != null && status.getPipelineIndex() != APRILTAG_PIPELINE_INDEX) {
            limelight.pipelineSwitch(APRILTAG_PIPELINE_INDEX);
            pipelineFixCount = pipelineFixCount + 1;
        }
    }

    /*
     * Call this ONCE per loop(). It reads the newest camera result and picks the target.
     */
    public void update() {
        checkPipeline();

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

        // Pass 1: for each cell, note whether we can see it and how high its HIGHEST tag is
        // (the largest ty). We only keep OUR alliance's cells.
        for (int cell = 0; cell < TOTAL_CELLS; cell++) {
            cellVisible[cell] = false;
            highestTyInCell[cell] = -Double.MAX_VALUE;
        }
        for (LLResultTypes.FiducialResult tag : tags) {
            int cell = getCellNumber(tag.getFiducialId());
            if (!isOurCell(cell)) {
                continue;
            }
            cellVisible[cell] = true;
            if (tag.getTargetYDegrees() > highestTyInCell[cell]) {
                highestTyInCell[cell] = tag.getTargetYDegrees();
            }
        }

        // Is our audience cell raised or lowered? This reads the same cell heights that Pass 1 just
        // filled in. It only REPORTS: it does not touch the lock or the target below.
        audienceCellStatus = findAudienceCellStatus();

        // The highest of OUR cells in view is the one we would pick with no lock at all.
        int highestCell = NO_CELL;
        for (int cell = 0; cell < TOTAL_CELLS; cell++) {
            if (cellVisible[cell]) {
                if (highestCell == NO_CELL || highestTyInCell[cell] > highestTyInCell[highestCell]) {
                    highestCell = cell;
                }
            }
        }

        // The locked cell is timed whenever it is in view, so we know how long it has been gone.
        if (lockedCell != NO_CELL && cellVisible[lockedCell]) {
            lockedCellUnseenTimer.reset();
        }

        int bestCell = chooseCell(highestCell);

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

        rawTxDegrees = txTotal / tagsInCell;
        txDegrees = rawTxDegrees + AIM_OFFSET_DEGREES;
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

    /*
     * Which side of our hive the target cell is on, for the screen: "FAR", "AUDIENCE", or "none"
     * when there is no target.
     */
    public String getTargetSideText() {
        if (targetCell == NO_CELL) {
            return "none";
        }
        if (targetCell == RED_FAR_CELL || targetCell == BLUE_FAR_CELL) {
            return "FAR";
        }
        return "AUDIENCE";
    }

    /*
     * Is our AUDIENCE-side cell UP (raised), DOWN (lowered) or UNKNOWN? Only UP and DOWN when the
     * camera sees BOTH of our cells (see "THE AUDIENCE CELL STATUS" at the top). UNKNOWN is a real
     * answer: do not treat it as DOWN.
     */
    public CellStatus getAudienceCellStatus() {
        return audienceCellStatus;
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
     * How many times the camera had to be switched back to the AprilTag pipeline during this run.
     * Above 0 means the camera restarted or lost its pipeline.
     */
    public int getPipelineFixCount() {
        return pipelineFixCount;
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
                telemetry.addData("Pipeline", "%d (wanted %d)   switched back %d time(s)",
                        status.getPipelineIndex(), APRILTAG_PIPELINE_INDEX, pipelineFixCount);
            } else {
                telemetry.addData("Limelight", "connected, no status yet");
            }
        }

        telemetry.addData("Vision State", state);
        telemetry.addData("Audience cell", "%s   (UP or DOWN needs BOTH of our cells in view)",
                audienceCellStatus);
        if (state == VisionState.TARGET_VISIBLE) {
            telemetry.addData("Aiming at", "%s %s side   (cell %d, %d tags used)",
                    alliance, getTargetSideText(), targetCell, tagCount);
            telemetry.addData("tx", "%.1f deg  (+ = target is to the RIGHT)", txDegrees);
            telemetry.addData("tx raw / offset", "%.1f deg camera,  %+.1f deg AIM_OFFSET",
                    rawTxDegrees, AIM_OFFSET_DEGREES);
            telemetry.addData("ty", "%.1f deg  (+ = up)", tyDegrees);
        }
        telemetry.addData("Result age", "%d ms", resultAgeMs);
    }

    // ==================================================================
    // PRIVATE HELPERS
    // ==================================================================

    /*
     * Picks the cell to aim at, using the lock. highestCell is the highest of OUR cells in view,
     * or NO_CELL if none is. Returns NO_CELL when we should report no target.
     */
    private int chooseCell(int highestCell) {
        if (highestCell == NO_CELL) {
            // None of our cells is in view. The lock stays, so a short dropout does not lose it.
            return NO_CELL;
        }

        if (lockedCell == NO_CELL) {
            // Nothing locked yet: take the highest cell and lock on it.
            lockedCell = highestCell;
            lockedCellUnseenTimer.reset();
            return lockedCell;
        }

        if (cellVisible[lockedCell]) {
            // The locked cell is in view. Switch only if the other cell is clearly higher.
            if (highestCell != lockedCell
                    && highestTyInCell[highestCell] > highestTyInCell[lockedCell] + CELL_SWITCH_MARGIN_DEGREES) {
                lockedCell = highestCell;
            }
            return lockedCell;
        }

        // The locked cell is out of view but our other cell is in view. Wait a moment before we
        // give up the lock, and report no target until then (so the turret holds its angle).
        if (lockedCellUnseenTimer.seconds() < LOCK_RELEASE_SECONDS) {
            return NO_CELL;
        }
        lockedCell = highestCell;
        lockedCellUnseenTimer.reset();
        return lockedCell;
    }

    /*
     * Decides whether our AUDIENCE cell is UP, DOWN or UNKNOWN, from the cell heights that Pass 1
     * of update() just filled in. See "THE AUDIENCE CELL STATUS" at the top for the rule.
     */
    private CellStatus findAudienceCellStatus() {
        // Which two cells are ours depends on the alliance.
        int audienceCell = RED_AUDIENCE_CELL;
        int farCell = RED_FAR_CELL;
        if (alliance == Alliance.BLUE) {
            audienceCell = BLUE_AUDIENCE_CELL;
            farCell = BLUE_FAR_CELL;
        }

        // A height only means something next to the other cell. With only one cell in view, ty
        // could be a raised cell far away or a lowered cell up close, so we cannot tell.
        if (!cellVisible[audienceCell] || !cellVisible[farCell]) {
            return CellStatus.UNKNOWN;
        }

        // Positive means the audience cell is HIGHER than the far cell (a bigger ty is higher).
        double audienceHeightAdvantage = highestTyInCell[audienceCell] - highestTyInCell[farCell];

        if (audienceHeightAdvantage >= CELL_SWITCH_MARGIN_DEGREES) {
            return CellStatus.UP;
        }
        if (audienceHeightAdvantage <= -CELL_SWITCH_MARGIN_DEGREES) {
            return CellStatus.DOWN;
        }

        // The two cells look about the same height, which is what a hive in the middle of a tip
        // looks like. Say so instead of guessing.
        return CellStatus.UNKNOWN;
    }

    private void clearTarget() {
        // With no fresh result, or fewer than two of our cells in view, we cannot compare heights.
        audienceCellStatus = CellStatus.UNKNOWN;

        state = VisionState.NO_TARGET;
        txDegrees = 0.0;
        rawTxDegrees = 0.0;
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
