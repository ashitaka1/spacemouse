// Package jog jogs an arm's end effector from an input_controller's six absolute axes.
package jog

import (
	"context"
	"fmt"
	"math"
	"sync"
	"sync/atomic"
	"time"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/components/gripper"
	"go.viam.com/rdk/components/input"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/generic"
	"go.viam.com/rdk/services/motion"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/rdk/utils"
)

// Model is the registered Viam model.
var Model = resource.NewModel("viam", "spacemouse", "jog")

const (
	defaultDeadband              = 0.05
	defaultTranslationMMPerSec   = 150.0
	defaultRotationDegPerSec     = 30.0
	defaultTickHz                = 50.0
	defaultMaxJointVelDegsPerSec = 90.0
	defaultMotionName            = "motion"
	defaultGripperButton         = "ButtonMenu"

	posDeadbandMM  = 1.0
	rotDeadbandDeg = 1.0
	commandTimeout = 500 * time.Millisecond
	errorCooldown  = 1 * time.Second

	// Cap virtual-goal lag so release-to-stop stays within these bounds.
	maxLagMM  = 50.0
	maxLagDeg = 20.0

	parkDrainTimeout = 500 * time.Millisecond
	parkDrainPoll    = 5 * time.Millisecond
)

// Config wires an arm to an input_controller.
type Config struct {
	ArmName    string `json:"arm"`
	InputName  string `json:"input_controller"`
	MotionName string `json:"motion,omitempty"`

	// Optional gripper toggled by button press (defaults to ButtonMenu).
	GripperName   string `json:"gripper,omitempty"`
	GripperButton string `json:"gripper_button,omitempty"`

	Deadband              *float64 `json:"deadband,omitempty"`
	TranslationMMPerSec   *float64 `json:"translation_mm_per_sec,omitempty"`
	RotationDegPerSec     *float64 `json:"rotation_deg_per_sec,omitempty"`
	TickHz                *float64 `json:"tick_hz,omitempty"`
	MaxJointVelDegsPerSec *float64 `json:"max_joint_vel_degs_per_sec,omitempty"`
	AutoStart             *bool    `json:"auto_start,omitempty"`
}

// Validate returns the deps the service requires.
func (c *Config) Validate(path string) ([]string, []string, error) {
	if c.ArmName == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "arm")
	}
	if c.InputName == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "input_controller")
	}
	motionName := c.MotionName
	if motionName == "" {
		motionName = defaultMotionName
	}
	deps := []string{
		arm.Named(c.ArmName).String(),
		input.Named(c.InputName).String(),
		motion.Named(motionName).String(),
	}
	if c.GripperName != "" {
		deps = append(deps, gripper.Named(c.GripperName).String())
	}
	return deps, nil, nil
}

func init() {
	resource.RegisterService(generic.API, Model,
		resource.Registration[resource.Resource, *Config]{
			Constructor: newJog,
		},
	)
}

type jog struct {
	resource.Named
	resource.AlwaysRebuild

	logger    logging.Logger
	arm       arm.Arm
	armName   string
	motionSvc motion.Service
	kinematic referenceframe.Model

	// Optional gripper + the button Control that toggles it.
	gripper       gripper.Gripper
	gripperButton input.Control
	gripperOpen   atomic.Bool

	deadband        float64
	transPerSec     float64
	rotPerSec       float64
	tickInterval    time.Duration
	maxJointVelRads float64

	// engageMu serializes start/stop against tick so stop returns only after
	// the arm is parked and no further goal will be sent.
	engageMu sync.Mutex
	engaged  atomic.Bool

	velMu      sync.RWMutex
	velocities [6]float64

	// movePending gates a single in-flight goal send; ticks drop otherwise.
	movePending atomic.Bool

	// pipelineOpen tracks whether we have an active teleop pipeline. We close
	// it on idle to avoid stale planning state leaking into the next session,
	// and reopen on the next active tick.
	pipelineOpen atomic.Bool

	moveMu       sync.Mutex
	moving       bool
	lastSentPose spatialmath.Pose
	errorUntil   time.Time
	// virtualGoal advances at translation_mm_per_sec regardless of arm position;
	// nil when idle, re-initialized from arm pose on next active tick.
	virtualGoal spatialmath.Pose

	cancelCtx  context.Context
	cancelFunc context.CancelFunc
	wg         sync.WaitGroup
}

func newJog(
	ctx context.Context,
	deps resource.Dependencies,
	conf resource.Config,
	logger logging.Logger,
) (resource.Resource, error) {
	cfg, err := resource.NativeConfig[*Config](conf)
	if err != nil {
		return nil, err
	}

	armRes, err := arm.FromProvider(deps, cfg.ArmName)
	if err != nil {
		return nil, fmt.Errorf("arm %q not found: %w", cfg.ArmName, err)
	}
	inRes, err := input.FromProvider(deps, cfg.InputName)
	if err != nil {
		return nil, fmt.Errorf("input_controller %q not found: %w", cfg.InputName, err)
	}
	motionName := cfg.MotionName
	if motionName == "" {
		motionName = defaultMotionName
	}
	motionSvc, err := motion.FromProvider(deps, motionName)
	if err != nil {
		return nil, fmt.Errorf("motion service %q not found: %w", motionName, err)
	}

	kinematic, err := armRes.Kinematics(ctx)
	if err != nil {
		return nil, fmt.Errorf("fetch arm kinematics: %w", err)
	}

	var gripperRes gripper.Gripper
	if cfg.GripperName != "" {
		gripperRes, err = gripper.FromProvider(deps, cfg.GripperName)
		if err != nil {
			return nil, fmt.Errorf("gripper %q not found: %w", cfg.GripperName, err)
		}
	}
	gripperButton := cfg.GripperButton
	if gripperButton == "" {
		gripperButton = defaultGripperButton
	}

	cancelCtx, cancelFunc := context.WithCancel(context.Background())
	j := &jog{
		Named:           conf.ResourceName().AsNamed(),
		logger:          logger,
		arm:             armRes,
		armName:         cfg.ArmName,
		motionSvc:       motionSvc,
		kinematic:       kinematic,
		gripper:         gripperRes,
		gripperButton:   input.Control(gripperButton),
		deadband:        firstNonNil(cfg.Deadband, defaultDeadband),
		transPerSec:     firstNonNil(cfg.TranslationMMPerSec, defaultTranslationMMPerSec),
		rotPerSec:       firstNonNil(cfg.RotationDegPerSec, defaultRotationDegPerSec),
		tickInterval:    tickInterval(firstNonNil(cfg.TickHz, defaultTickHz)),
		maxJointVelRads: utils.DegToRad(firstNonNil(cfg.MaxJointVelDegsPerSec, defaultMaxJointVelDegsPerSec)),
		cancelCtx:       cancelCtx,
		cancelFunc:      cancelFunc,
	}
	j.engaged.Store(firstNonNil(cfg.AutoStart, true))

	if err := j.subscribe(ctx, inRes); err != nil {
		cancelFunc()
		return nil, fmt.Errorf("subscribe to input_controller: %w", err)
	}

	if j.engaged.Load() {
		if err := j.openPipeline(ctx); err != nil {
			cancelFunc()
			return nil, fmt.Errorf("open pipeline: %w", err)
		}
	}

	j.wg.Add(1)
	go j.drainLoop()

	logger.CInfow(ctx, "jog service started",
		"arm", cfg.ArmName,
		"input_controller", cfg.InputName,
		"motion", motionName,
		"tick_interval_ms", j.tickInterval.Milliseconds(),
		"translation_mm_per_sec", j.transPerSec,
		"rotation_deg_per_sec", j.rotPerSec,
		"engaged", j.engaged.Load(),
	)
	return j, nil
}

func poseInFrameJSON(pose spatialmath.Pose) string {
	pt := pose.Point()
	ov := pose.Orientation().OrientationVectorDegrees()
	return fmt.Sprintf(
		`{"reference_frame":"world","pose":{"x":%f,"y":%f,"z":%f,"o_x":%f,"o_y":%f,"o_z":%f,"theta":%f}}`,
		pt.X, pt.Y, pt.Z, ov.OX, ov.OY, ov.OZ, ov.Theta,
	)
}

// armPoseInWorld returns the arm EE pose in world frame, regardless of where
// the arm is placed. arm.EndPosition returns arm-local coords which breaks
// the planner's reach check for arms off-origin; motion.GetPose handles the
// framesystem transform for us.
func (j *jog) armPoseInWorld(ctx context.Context) (spatialmath.Pose, error) {
	pif, err := j.motionSvc.GetPose(ctx, j.armName, referenceframe.World, nil, nil)
	if err != nil {
		return nil, err
	}
	return pif.Pose(), nil
}

func (j *jog) openPipeline(ctx context.Context) error {
	currentPose, err := j.armPoseInWorld(ctx)
	if err != nil {
		return fmt.Errorf("read current pose: %w", err)
	}
	startReq := fmt.Sprintf(
		`{"name":"builtin","component_name":"%s","destination":%s}`,
		j.armName, poseInFrameJSON(currentPose),
	)
	cmdCtx, cancel := context.WithTimeout(ctx, commandTimeout)
	defer cancel()
	if _, err = j.motionSvc.DoCommand(cmdCtx, map[string]interface{}{
		"teleop_start": startReq,
	}); err != nil {
		return err
	}
	j.pipelineOpen.Store(true)
	return nil
}

func (j *jog) sendGoal(ctx context.Context, pose spatialmath.Pose) error {
	cmdCtx, cancel := context.WithTimeout(ctx, commandTimeout)
	defer cancel()
	_, err := j.motionSvc.DoCommand(cmdCtx, map[string]interface{}{
		"teleop_move":    poseInFrameJSON(pose),
		"component_name": j.armName,
	})
	return err
}

func (j *jog) closePipeline(ctx context.Context) {
	cmdCtx, cancel := context.WithTimeout(ctx, commandTimeout)
	defer cancel()
	if _, err := j.motionSvc.DoCommand(cmdCtx, map[string]interface{}{"teleop_stop": ""}); err != nil {
		j.logger.CWarnw(cmdCtx, "close pipeline failed", "err", err)
	}
	j.pipelineOpen.Store(false)
}

// DoCommand accepts {"command": "start" | "stop" | "status"} and returns the
// engagement status. Without a command it returns the current velocities.
func (j *jog) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	name, ok := cmd["command"]
	if !ok {
		return map[string]interface{}{"velocities": j.velocityMap()}, nil
	}
	switch name {
	case "start":
		j.start()
	case "stop":
		j.stop(ctx)
	case "status":
	default:
		return nil, fmt.Errorf("unknown command %v", name)
	}
	return j.status(), nil
}

func (j *jog) start() {
	j.engageMu.Lock()
	defer j.engageMu.Unlock()
	j.engaged.Store(true)
}

func (j *jog) stop(ctx context.Context) {
	j.engageMu.Lock()
	defer j.engageMu.Unlock()
	j.engaged.Store(false)

	j.moveMu.Lock()
	wasMoving := j.moving
	j.moving = false
	j.virtualGoal = nil
	j.moveMu.Unlock()

	if wasMoving {
		j.parkArm()
	} else if j.pipelineOpen.Load() {
		j.closePipeline(ctx)
	}
}

func (j *jog) status() map[string]interface{} {
	return map[string]interface{}{
		"running":    j.engaged.Load(),
		"velocities": j.velocityMap(),
	}
}

func (j *jog) velocityMap() map[string]float64 {
	v := j.currentVelocities()
	return map[string]float64{
		"x":  v[0],
		"y":  v[1],
		"z":  v[2],
		"rx": v[3],
		"ry": v[4],
		"rz": v[5],
	}
}

func (j *jog) Close(ctx context.Context) error {
	j.cancelFunc()
	j.wg.Wait()
	if j.pipelineOpen.Load() {
		j.closePipeline(ctx)
	}
	return nil
}

func firstNonNil[T any](p *T, def T) T {
	if p == nil {
		return def
	}
	return *p
}

func tickInterval(hz float64) time.Duration {
	if hz <= 0 {
		hz = defaultTickHz
	}
	d := time.Duration(math.Round(float64(time.Second) / hz))
	if d < time.Millisecond {
		d = time.Millisecond
	}
	return d
}
