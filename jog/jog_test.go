package jog

import (
	"context"
	"sync"
	"testing"
	"time"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/components/input"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/services/motion"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/rdk/testutils/inject"
	injectmotion "go.viam.com/rdk/testutils/inject/motion"
	"go.viam.com/test"
)

func fiftyHz(t *testing.T, deadband float64) *jog {
	t.Helper()
	ctx, cancel := context.WithCancel(context.Background())
	t.Cleanup(cancel)
	return &jog{
		logger:       logging.NewTestLogger(t),
		deadband:     deadband,
		transPerSec:  100.0,
		rotPerSec:    45.0,
		tickInterval: 20 * time.Millisecond,
		cancelCtx:    ctx,
		cancelFunc:   cancel,
	}
}

func TestAnyAboveDeadband(t *testing.T) {
	j := fiftyHz(t, 0.1)
	test.That(t, j.anyAxisAboveDeadband([6]float64{}), test.ShouldBeFalse)
	test.That(t, j.anyAxisAboveDeadband([6]float64{0.05, 0.05, 0.05, 0.05, 0.05, 0.05}), test.ShouldBeFalse)
	test.That(t, j.anyAxisAboveDeadband([6]float64{0, 0, 0, 0, 0, 0.2}), test.ShouldBeTrue)
	test.That(t, j.anyAxisAboveDeadband([6]float64{-0.2, 0, 0, 0, 0, 0}), test.ShouldBeTrue)
}

func TestApplyDelta_TranslationOnly(t *testing.T) {
	start := spatialmath.NewPose(
		r3.Vector{X: 100, Y: 200, Z: 300},
		&spatialmath.R4AA{Theta: 0, RX: 1},
	)
	out, err := applyDelta(start, poseDeltaStep{X: 5, Y: -3, Z: 2})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, out.Point().X, test.ShouldAlmostEqual, 105.0)
	test.That(t, out.Point().Y, test.ShouldAlmostEqual, 197.0)
	test.That(t, out.Point().Z, test.ShouldAlmostEqual, 302.0)
	// orientation unchanged
	oriBefore := start.Orientation().AxisAngles()
	oriAfter := out.Orientation().AxisAngles()
	test.That(t, oriAfter.Theta, test.ShouldAlmostEqual, oriBefore.Theta)
}

func TestApplyDelta_RotationChangesOrientation(t *testing.T) {
	start := spatialmath.NewPose(
		r3.Vector{X: 100, Y: 200, Z: 300},
		&spatialmath.R4AA{Theta: 0, RX: 1},
	)
	out, err := applyDelta(start, poseDeltaStep{RZ: 10})
	test.That(t, err, test.ShouldBeNil)
	// position unchanged
	test.That(t, out.Point().X, test.ShouldAlmostEqual, 100.0)
	// orientation theta should now be ~10 degrees in radians
	ori := out.Orientation().AxisAngles()
	test.That(t, ori.Theta, test.ShouldAlmostEqual, 10.0*3.141592653589793/180.0, 1e-9)
}

func TestTick_BelowDeadbandSkipsArmCalls(t *testing.T) {
	j := fiftyHz(t, 0.1)
	a := &inject.Arm{}
	var endPosCalls, moveCalls int
	a.EndPositionFunc = func(context.Context, map[string]interface{}) (spatialmath.Pose, error) {
		endPosCalls++
		return nil, nil
	}
	a.MoveToPositionFunc = func(context.Context, spatialmath.Pose, map[string]interface{}) error {
		moveCalls++
		return nil
	}
	j.arm = a

	j.velocities = [6]float64{0.05, 0.05, 0.05, 0.05, 0.05, 0.05}
	j.tick()

	test.That(t, endPosCalls, test.ShouldEqual, 0)
	test.That(t, moveCalls, test.ShouldEqual, 0)
}

func TestSubscribe_CallbackUpdatesVelocity(t *testing.T) {
	j := fiftyHz(t, 0.0)
	fake := &fakeInput{callbacks: map[axisKey]input.ControlFunction{}}
	err := j.subscribe(context.Background(), fake)
	test.That(t, err, test.ShouldBeNil)

	fake.fire(input.AbsoluteX, input.PositionChangeAbs, 0.42)
	fake.fire(input.AbsoluteRZ, input.PositionChangeAbs, -0.7)

	v := j.currentVelocities()
	test.That(t, v[0], test.ShouldAlmostEqual, 0.42)
	test.That(t, v[5], test.ShouldAlmostEqual, -0.7)
}

func TestSubscribe_DisconnectZeroes(t *testing.T) {
	j := fiftyHz(t, 0.0)
	fake := &fakeInput{callbacks: map[axisKey]input.ControlFunction{}}
	err := j.subscribe(context.Background(), fake)
	test.That(t, err, test.ShouldBeNil)

	fake.fire(input.AbsoluteX, input.PositionChangeAbs, 0.9)
	test.That(t, j.currentVelocities()[0], test.ShouldAlmostEqual, 0.9)

	fake.fire(input.AbsoluteX, input.Disconnect, 0.0)
	test.That(t, j.currentVelocities(), test.ShouldEqual, [6]float64{})
}

func TestPoseDiffExceeds(t *testing.T) {
	ori := &spatialmath.R4AA{Theta: 0, RX: 1}
	a := spatialmath.NewPose(r3.Vector{X: 100, Y: 200, Z: 300}, ori)

	t.Run("identical poses do not exceed", func(t *testing.T) {
		b := spatialmath.NewPose(r3.Vector{X: 100, Y: 200, Z: 300}, ori)
		test.That(t, poseDiffExceeds(a, b, 1.0, 1.0), test.ShouldBeFalse)
	})

	t.Run("sub-mm translation does not exceed 1mm threshold", func(t *testing.T) {
		b := spatialmath.NewPose(r3.Vector{X: 100.5, Y: 200.5, Z: 300.5}, ori)
		test.That(t, poseDiffExceeds(a, b, 1.0, 1.0), test.ShouldBeFalse)
	})

	t.Run("2mm translation exceeds 1mm threshold", func(t *testing.T) {
		b := spatialmath.NewPose(r3.Vector{X: 102, Y: 200, Z: 300}, ori)
		test.That(t, poseDiffExceeds(a, b, 1.0, 1.0), test.ShouldBeTrue)
	})

	t.Run("5-degree rotation exceeds 1-degree threshold", func(t *testing.T) {
		b := spatialmath.NewPose(r3.Vector{X: 100, Y: 200, Z: 300},
			&spatialmath.R4AA{Theta: 5 * 3.141592653589793 / 180, RX: 1})
		test.That(t, poseDiffExceeds(a, b, 1.0, 1.0), test.ShouldBeTrue)
	})

	t.Run("sub-threshold rotation does not exceed", func(t *testing.T) {
		b := spatialmath.NewPose(r3.Vector{X: 100, Y: 200, Z: 300},
			&spatialmath.R4AA{Theta: 0.1 * 3.141592653589793 / 180, RX: 1})
		test.That(t, poseDiffExceeds(a, b, 1.0, 1.0), test.ShouldBeFalse)
	})
}

func TestTick_IdleToIdleIsNoop(t *testing.T) {
	// When velocity is below deadband and we haven't been moving, tick should
	// read nothing from the arm and send no commands.
	j := fiftyHz(t, 0.1)
	a := &inject.Arm{}
	var endPosCalls int
	a.EndPositionFunc = func(context.Context, map[string]interface{}) (spatialmath.Pose, error) {
		endPosCalls++
		return nil, nil
	}
	j.arm = a
	j.velocities = [6]float64{0.05, 0.05, 0.05, 0.05, 0.05, 0.05} // all below 0.1 deadband
	j.tick()
	test.That(t, endPosCalls, test.ShouldEqual, 0)
	test.That(t, j.moving, test.ShouldBeFalse)
}

func TestTickInterval(t *testing.T) {
	test.That(t, tickInterval(50), test.ShouldEqual, 20*time.Millisecond)
	test.That(t, tickInterval(0), test.ShouldEqual, 20*time.Millisecond)     // defaults
	test.That(t, tickInterval(10_000), test.ShouldEqual, 1*time.Millisecond) // floor at 1ms
	test.That(t, tickInterval(100), test.ShouldEqual, 10*time.Millisecond)
}

func TestFirstNonNil(t *testing.T) {
	test.That(t, firstNonNil(nil, 42.0), test.ShouldEqual, 42.0)
	v := 7.0
	test.That(t, firstNonNil(&v, 42.0), test.ShouldEqual, 7.0)
}

func TestConfigValidate(t *testing.T) {
	_, _, err := (&Config{}).Validate("/foo")
	test.That(t, err, test.ShouldNotBeNil)

	_, _, err = (&Config{ArmName: "a"}).Validate("/foo")
	test.That(t, err, test.ShouldNotBeNil)

	deps, _, err := (&Config{ArmName: "my-arm", InputName: "my-sm"}).Validate("/foo")
	test.That(t, err, test.ShouldBeNil)
	test.That(t, deps, test.ShouldResemble, []string{
		arm.Named("my-arm").String(),
		input.Named("my-sm").String(),
		motion.Named(defaultMotionName).String(),
	})
}

func TestConfig_AutoStartDefaultsTrue(t *testing.T) {
	test.That(t, firstNonNil((&Config{}).AutoStart, true), test.ShouldBeTrue)
	off := false
	test.That(t, firstNonNil((&Config{AutoStart: &off}).AutoStart, true), test.ShouldBeFalse)
}

func TestTick_StoppedSkipsArmCalls(t *testing.T) {
	j, rec, inputsCalls := wiredJog(t)
	j.engaged.Store(false)
	j.velocities = [6]float64{0.5}

	j.tick()

	test.That(t, rec.commands(), test.ShouldBeEmpty)
	test.That(t, inputsCalls(), test.ShouldEqual, 0)
}

func TestStop_WhileMovingParksAndCloses(t *testing.T) {
	j, rec, _ := wiredJog(t)
	j.velocities = [6]float64{0.5}

	j.tick()
	j.stop(context.Background())

	cmds := rec.commands()
	test.That(t, cmds[0], test.ShouldEqual, "teleop_move")
	test.That(t, cmds[len(cmds)-1], test.ShouldEqual, "teleop_stop")
	test.That(t, j.pipelineOpen.Load(), test.ShouldBeFalse)

	j.tick()
	test.That(t, rec.commands(), test.ShouldResemble, cmds)
}

func TestStart_ReopensPipelineOnNextDeflection(t *testing.T) {
	j, rec, _ := wiredJog(t)
	j.engaged.Store(false)
	j.pipelineOpen.Store(false)
	j.velocities = [6]float64{0.5}

	_, err := j.DoCommand(context.Background(), map[string]interface{}{"command": "start"})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, rec.commands(), test.ShouldBeEmpty)

	j.tick()
	test.That(t, rec.commands()[0], test.ShouldEqual, "teleop_start")
}

func TestStartStop_Idempotent(t *testing.T) {
	j, rec, _ := wiredJog(t)
	ctx := context.Background()

	for range 2 {
		resp, err := j.DoCommand(ctx, map[string]interface{}{"command": "start"})
		test.That(t, err, test.ShouldBeNil)
		test.That(t, resp["running"], test.ShouldBeTrue)
	}
	for range 2 {
		resp, err := j.DoCommand(ctx, map[string]interface{}{"command": "stop"})
		test.That(t, err, test.ShouldBeNil)
		test.That(t, resp["running"], test.ShouldBeFalse)
	}
	test.That(t, rec.commands(), test.ShouldResemble, []string{"teleop_stop"})

	resp, err := j.DoCommand(ctx, map[string]interface{}{"command": "status"})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, resp["running"], test.ShouldBeFalse)
	test.That(t, resp, test.ShouldContainKey, "velocities")
}

func TestDoCommand_UnknownCommandErrors(t *testing.T) {
	j, _, _ := wiredJog(t)
	_, err := j.DoCommand(context.Background(), map[string]interface{}{"command": "bogus"})
	test.That(t, err, test.ShouldNotBeNil)
}

func TestDoCommand_NoCommandReturnsVelocities(t *testing.T) {
	j, _, _ := wiredJog(t)
	j.velocities = [6]float64{0.25}
	resp, err := j.DoCommand(context.Background(), map[string]interface{}{})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, resp, test.ShouldNotContainKey, "running")
	test.That(t, resp["velocities"].(map[string]float64)["x"], test.ShouldEqual, 0.25)
}

func TestClose_WhileStoppedSendsNoTeleopStop(t *testing.T) {
	j, rec, _ := wiredJog(t)
	j.stop(context.Background())
	before := rec.commands()

	test.That(t, j.Close(context.Background()), test.ShouldBeNil)
	test.That(t, rec.commands(), test.ShouldResemble, before)
}

func TestGripperButton_OnlyWhileEngaged(t *testing.T) {
	j := fiftyHz(t, 0.1)
	g := &inject.Gripper{}
	var opens, grabs int
	g.OpenFunc = func(context.Context, map[string]interface{}) error {
		opens++
		return nil
	}
	g.GrabFunc = func(context.Context, map[string]interface{}) (bool, error) {
		grabs++
		return true, nil
	}
	j.gripper = g
	j.gripperButton = input.ButtonMenu
	fake := &fakeInput{callbacks: map[axisKey]input.ControlFunction{}}
	test.That(t, j.subscribe(context.Background(), fake), test.ShouldBeNil)

	fake.fire(input.ButtonMenu, input.ButtonPress, 1)
	test.That(t, opens+grabs, test.ShouldEqual, 0)

	j.start()
	fake.fire(input.ButtonMenu, input.ButtonPress, 1)
	test.That(t, opens, test.ShouldEqual, 1)
	fake.fire(input.ButtonMenu, input.ButtonPress, 1)
	test.That(t, grabs, test.ShouldEqual, 1)
}

// wiredJog returns an engaged jog with an open pipeline, a 1-DoF arm, and a
// motion service that records the key of every DoCommand it receives.
func wiredJog(t *testing.T) (*jog, *commandRecorder, func() int) {
	t.Helper()
	j := fiftyHz(t, 0.1)
	j.armName = "arm"

	joint, err := referenceframe.NewRotationalFrame("j0", spatialmath.R4AA{RZ: 1}, referenceframe.Limit{Min: -3, Max: 3})
	test.That(t, err, test.ShouldBeNil)
	link, err := referenceframe.NewStaticFrame("l0", spatialmath.NewPoseFromPoint(r3.Vector{X: 300}))
	test.That(t, err, test.ShouldBeNil)
	j.kinematic, err = referenceframe.NewSerialModel("m", []referenceframe.Frame{joint, link})
	test.That(t, err, test.ShouldBeNil)

	var inputsMu sync.Mutex
	var inputsCalls int
	a := &inject.Arm{}
	a.CurrentInputsFunc = func(context.Context) ([]referenceframe.Input, error) {
		inputsMu.Lock()
		defer inputsMu.Unlock()
		inputsCalls++
		return []referenceframe.Input{0}, nil
	}
	j.arm = a

	rec := &commandRecorder{}
	ms := injectmotion.NewMotionService("motion")
	ms.GetPoseFunc = func(context.Context, string, string, []*referenceframe.LinkInFrame, map[string]interface{}) (*referenceframe.PoseInFrame, error) {
		return referenceframe.NewPoseInFrame(referenceframe.World, spatialmath.NewPoseFromPoint(r3.Vector{X: 300})), nil
	}
	ms.DoCommandFunc = func(_ context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
		for k := range cmd {
			if k != "component_name" {
				rec.add(k)
			}
		}
		return nil, nil
	}
	j.motionSvc = ms

	j.engaged.Store(true)
	j.pipelineOpen.Store(true)
	return j, rec, func() int {
		inputsMu.Lock()
		defer inputsMu.Unlock()
		return inputsCalls
	}
}

type commandRecorder struct {
	mu   sync.Mutex
	keys []string
}

func (r *commandRecorder) add(k string) {
	r.mu.Lock()
	defer r.mu.Unlock()
	r.keys = append(r.keys, k)
}

func (r *commandRecorder) commands() []string {
	r.mu.Lock()
	defer r.mu.Unlock()
	return append([]string(nil), r.keys...)
}

// fakeInput is a minimal input.Controller stand-in that lets tests fire events.
type axisKey struct {
	control input.Control
	event   input.EventType
}

type fakeInput struct {
	input.Controller
	mu        sync.Mutex
	callbacks map[axisKey]input.ControlFunction
}

func (f *fakeInput) RegisterControlCallback(
	_ context.Context,
	control input.Control,
	triggers []input.EventType,
	cb input.ControlFunction,
	_ map[string]interface{},
) error {
	f.mu.Lock()
	defer f.mu.Unlock()
	for _, trig := range triggers {
		f.callbacks[axisKey{control, trig}] = cb
	}
	return nil
}

func (f *fakeInput) fire(control input.Control, event input.EventType, value float64) {
	f.mu.Lock()
	cb := f.callbacks[axisKey{control, event}]
	f.mu.Unlock()
	if cb != nil {
		cb(context.Background(), input.Event{
			Control: control, Event: event, Value: value,
		})
	}
}

// pin unused imports so the file compiles cleanly without "declared but not used" noise.
var _ = input.AbsoluteX
