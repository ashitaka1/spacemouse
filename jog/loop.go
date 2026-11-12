package jog

import (
	"context"
	"fmt"
	"math"
	"strings"
	"time"

	"go.viam.com/rdk/components/input"
	"go.viam.com/rdk/referenceframe"
)

var axisControls = [6]input.Control{
	input.AbsoluteX,
	input.AbsoluteY,
	input.AbsoluteZ,
	input.AbsoluteRX,
	input.AbsoluteRY,
	input.AbsoluteRZ,
}

func (j *jog) subscribe(ctx context.Context, ctrl input.Controller) error {
	for i, control := range axisControls {
		idx := i
		cb := func(_ context.Context, ev input.Event) {
			j.velMu.Lock()
			j.velocities[idx] = ev.Value
			j.velMu.Unlock()
		}
		err := ctrl.RegisterControlCallback(ctx, control,
			[]input.EventType{input.PositionChangeAbs}, cb, nil)
		if err != nil {
			return err
		}
	}

	// Zero velocities on disconnect so the arm stops if the controller drops.
	disc := func(_ context.Context, _ input.Event) {
		j.velMu.Lock()
		j.velocities = [6]float64{}
		j.velMu.Unlock()
	}
	for _, control := range axisControls {
		if err := ctrl.RegisterControlCallback(ctx, control,
			[]input.EventType{input.Disconnect}, disc, nil); err != nil {
			return err
		}
	}

	if j.gripper != nil {
		if err := ctrl.RegisterControlCallback(ctx, j.gripperButton,
			[]input.EventType{input.ButtonPress}, j.toggleGripper, nil); err != nil {
			return fmt.Errorf("register gripper button %q: %w", j.gripperButton, err)
		}
	}
	return nil
}

func (j *jog) toggleGripper(ctx context.Context, _ input.Event) {
	if !j.engaged.Load() {
		return
	}
	opening := !j.gripperOpen.Load()
	if opening {
		if err := j.gripper.Open(ctx, nil); err != nil {
			j.logger.CWarnw(ctx, "gripper open failed", "err", err)
			return
		}
	} else {
		if _, err := j.gripper.Grab(ctx, nil); err != nil {
			j.logger.CWarnw(ctx, "gripper grab failed", "err", err)
			return
		}
	}
	j.gripperOpen.Store(opening)
}

func (j *jog) drainLoop() {
	defer j.wg.Done()
	ticker := time.NewTicker(j.tickInterval)
	defer ticker.Stop()

	for {
		select {
		case <-j.cancelCtx.Done():
			return
		case <-ticker.C:
			j.tick()
		}
	}
}

func (j *jog) tick() {
	j.engageMu.Lock()
	defer j.engageMu.Unlock()

	if !j.engaged.Load() {
		return
	}

	vel := j.currentVelocities()
	above := j.anyAxisAboveDeadband(vel)

	if !above {
		// Clear virtualGoal on every idle tick so a stale goal can't carry over.
		j.moveMu.Lock()
		wasMoving := j.moving
		j.moving = false
		j.virtualGoal = nil
		j.moveMu.Unlock()
		if !wasMoving {
			return
		}
		j.parkArm()
		return
	}

	j.moveMu.Lock()
	cooldownUntil := j.errorUntil
	j.moveMu.Unlock()
	if time.Now().Before(cooldownUntil) {
		return
	}

	if !j.movePending.CompareAndSwap(false, true) {
		return
	}

	// Re-open the pipeline on idle→active transition. parkArm closed it so
	// stale planning state wouldn't leak into the next activity burst.
	if !j.pipelineOpen.Load() {
		if err := j.openPipeline(j.cancelCtx); err != nil {
			j.movePending.Store(false)
			j.logger.CWarnw(j.cancelCtx, "pipeline open on activation failed", "err", err)
			return
		}
	}

	currentPose, err := j.armPoseInWorld(j.cancelCtx)
	if err != nil {
		j.movePending.Store(false)
		j.logger.CWarnw(j.cancelCtx, "arm EndPosition failed", "err", err)
		return
	}
	currentInputs, err := j.arm.CurrentInputs(j.cancelCtx)
	if err != nil {
		j.movePending.Store(false)
		j.logger.CWarnw(j.cancelCtx, "arm CurrentInputs failed", "err", err)
		return
	}

	j.moveMu.Lock()
	base := j.virtualGoal
	j.moveMu.Unlock()
	if base == nil {
		base = currentPose
	}

	dtSec := j.tickInterval.Seconds()
	delta := poseDeltaStep{
		X:  vel[0] * j.transPerSec * dtSec,
		Y:  vel[1] * j.transPerSec * dtSec,
		Z:  vel[2] * j.transPerSec * dtSec,
		RX: vel[3] * j.rotPerSec * dtSec,
		RY: vel[4] * j.rotPerSec * dtSec,
		RZ: vel[5] * j.rotPerSec * dtSec,
	}

	// Scale translation and rotation independently against max_joint_vel so a
	// wrist singularity doesn't choke translation. When scaled, reset base to
	// currentPose so goal doesn't drift ahead in joint space.
	scaleT, scaleR, maxJointTrans, maxJointRot := j.scaleForJointLimitSplit(currentInputs, delta, dtSec)
	if scaleT < 1.0 || scaleR < 1.0 {
		j.logger.CInfof(j.cancelCtx,
			"jacobian scale_t=%.3f scale_r=%.3f joint_trans=%.3frad joint_rot=%.3frad budget=%.3frad inputs=%v",
			scaleT, scaleR, maxJointTrans, maxJointRot, j.maxJointVelRads*dtSec, inputsAsSlice(currentInputs))
		delta.X *= scaleT
		delta.Y *= scaleT
		delta.Z *= scaleT
		delta.RX *= scaleR
		delta.RY *= scaleR
		delta.RZ *= scaleR
		base = currentPose
	}

	goal, err := applyDelta(base, delta)
	if err != nil {
		j.movePending.Store(false)
		j.logger.CWarnw(j.cancelCtx, "pose composition failed", "err", err)
		return
	}

	if poseDiffExceeds(currentPose, goal, maxLagMM, maxLagDeg) {
		goal = currentPose
	}

	j.moveMu.Lock()
	j.virtualGoal = goal
	lastSent := j.lastSentPose
	j.moveMu.Unlock()
	if lastSent != nil && !poseDiffExceeds(lastSent, goal, posDeadbandMM, rotDeadbandDeg) {
		j.movePending.Store(false)
		return
	}

	j.moveMu.Lock()
	j.moving = true
	j.moveMu.Unlock()

	go func() {
		defer j.movePending.Store(false)
		if err := j.sendGoal(j.cancelCtx, goal); err != nil {
			// Motion service drops the teleop pipeline on any reconfigure (RDK quirk).
			// Re-open and let the next tick retry.
			if strings.Contains(err.Error(), "not running") {
				if openErr := j.openPipeline(j.cancelCtx); openErr != nil {
					j.logger.CWarnw(j.cancelCtx, "pipeline re-open failed", "err", openErr)
					j.moveMu.Lock()
					j.errorUntil = time.Now().Add(errorCooldown)
					j.moveMu.Unlock()
				}
				return
			}
			j.logger.CWarnw(j.cancelCtx, "send goal failed", "err", err)
			j.moveMu.Lock()
			j.errorUntil = time.Now().Add(errorCooldown)
			j.moveMu.Unlock()
			return
		}
		j.moveMu.Lock()
		j.lastSentPose = goal
		j.moveMu.Unlock()
	}()
}

func (j *jog) parkArm() {
	// Park must always fire; drain in-flight so arm doesn't chase a stale goal.
	deadline := time.Now().Add(parkDrainTimeout)
	for !j.movePending.CompareAndSwap(false, true) {
		if time.Now().After(deadline) {
			j.logger.CWarn(j.cancelCtx, "park: in-flight send did not drain; sending anyway")
			break
		}
		time.Sleep(parkDrainPoll)
	}
	defer j.movePending.Store(false)

	defer func() {
		j.moveMu.Lock()
		j.virtualGoal = nil
		j.moveMu.Unlock()
		// Close the pipeline so stale planning state doesn't leak into the
		// next activity burst. Next active tick re-opens fresh.
		j.closePipeline(j.cancelCtx)
	}()

	pose, err := j.armPoseInWorld(j.cancelCtx)
	if err != nil {
		j.logger.CWarnw(j.cancelCtx, "arm EndPosition failed on park", "err", err)
		return
	}
	if err := j.sendGoal(j.cancelCtx, pose); err != nil {
		j.logger.CWarnw(j.cancelCtx, "park arm failed", "err", err)
		return
	}
	j.moveMu.Lock()
	j.lastSentPose = pose
	j.moveMu.Unlock()
}

func (j *jog) currentVelocities() [6]float64 {
	j.velMu.RLock()
	defer j.velMu.RUnlock()
	return j.velocities
}

func (j *jog) anyAxisAboveDeadband(vel [6]float64) bool {
	for _, v := range vel {
		if math.Abs(v) > j.deadband {
			return true
		}
	}
	return false
}

// scaleForJointLimitSplit returns (scaleTrans, scaleRot, maxJointTrans,
// maxJointRot). Translation and rotation are scaled independently so a
// singularity in one axis doesn't choke the other. Both scales are 1.0 if
// Jacobian/DLS fails.
func (j *jog) scaleForJointLimitSplit(inputs []referenceframe.Input, d poseDeltaStep, dtSec float64) (float64, float64, float64, float64) {
	J, err := jacobianAt(j.kinematic, inputs)
	if err != nil {
		j.logger.CWarnw(j.cancelCtx, "jacobian compute failed", "err", err)
		return 1.0, 1.0, 0, 0
	}
	transTCP := []float64{d.X, d.Y, d.Z, 0, 0, 0}
	rotTCP := []float64{
		0, 0, 0,
		d.RX * math.Pi / 180, d.RY * math.Pi / 180, d.RZ * math.Pi / 180,
	}

	jointTrans, err := solveDLS(J, transTCP, dlsLambda)
	if err != nil {
		j.logger.CWarnw(j.cancelCtx, "DLS solve failed (trans)", "err", err)
		return 1.0, 1.0, 0, 0
	}
	jointRot, err := solveDLS(J, rotTCP, dlsLambda)
	if err != nil {
		j.logger.CWarnw(j.cancelCtx, "DLS solve failed (rot)", "err", err)
		return 1.0, 1.0, 0, 0
	}
	return jointVelScale(jointTrans, j.maxJointVelRads, dtSec),
		jointVelScale(jointRot, j.maxJointVelRads, dtSec),
		maxAbs(jointTrans), maxAbs(jointRot)
}

func maxAbs(xs []float64) float64 {
	m := 0.0
	for _, x := range xs {
		if a := math.Abs(x); a > m {
			m = a
		}
	}
	return m
}

func inputsAsSlice(inputs []referenceframe.Input) []float64 {
	out := make([]float64, len(inputs))
	for i, in := range inputs {
		out[i] = float64(in)
	}
	return out
}
