using Godot;
using System.Net.Sockets;
using System.Text;
using System.Text.Json;

namespace VehicleDynamics
{
  // =================================================================================
  // COMPONENT: INTEGRAL-BOUNDED PID CONTROLLER
  // =================================================================================
  public class PIDController
  {
    public float Kp, Ki, Kd;
    private float _integralAccumulator;
    private float _integralLimit;

    public PIDController(float p, float i, float d, float integralLimit = 20000.0f)
    {
      Kp = p; Ki = i; Kd = d; _integralLimit = integralLimit;
    }

    public float Update(float error, float currentVelocity, float dt)
    {
      if (dt <= 0.0001f) return 0f;

      if (Mathf.Abs(error) < 1.0f)
      {
        _integralAccumulator += error * dt;
        _integralAccumulator = Mathf.Clamp(_integralAccumulator, -_integralLimit, _integralLimit);
      }
      else
      {
        _integralAccumulator *= 0.9f;
      }

      float pTerm = Kp * error;
      float iTerm = Ki * _integralAccumulator;
      float dTerm = Kd * (0f - currentVelocity);

      return pTerm + iTerm + dTerm;
    }

    public void Reset() => _integralAccumulator = 0f;
  }

  // =================================================================================
  // EXTERNAL ENTITIES: GRAVITY WELL
  // =================================================================================
  public partial class GravityWell : Node3D
  {
    [Export] public float Mass = 1500000.0f;
  }

  // =================================================================================
  // MAIN CLASS: RELATIVISTIC OMNI-HYBRID VEHICLE
  // =================================================================================
  [GlobalClass]
  public partial class RoverDrone : Node3D
  {
    public enum VehicleMode { Car, TransitioningToDrone, Drone, TransitioningToCar }

    [ExportGroup("Hybrid State")]
    [Export] public VehicleMode CurrentMode = VehicleMode.Car;
    [Export] public float TransitionDuration = 0.8f;

    // --- Car Tuning Parameters ---
    [ExportGroup("Suspension Physics")]
    [Export] public float RestLength = 0.7f;
    [Export] public float SpringTravel = 0.4f;
    [Export] public float BaseSpringStiffness = 45000.0f;
    [Export] public float ProgressiveStiffness = 85000.0f;
    [Export] public float SpringDamping = 4500.0f;
    [Export] public float WheelRadius = 0.35f;

    [ExportGroup("Vehicle Dynamics")]
    [Export] public float EngineForce = 45000.0f;
    [Export] public float MaxSteerAngle = 0.5f;
    [Export] public float GripFriction = 12.0f;
    [Export] public float Mass = 2500.0f;
    [Export] public float WheelBase = 3.0f;
    [Export] public float TrackWidth = 2.2f;

    // --- Drone Tuning Parameters ---
    [ExportGroup("Quadcopter Dynamics")]
    [Export] public float MaxMotorThrust = 40000.0f;
    [Export] public float YawDragFactor = 0.8f;
    [Export] public float DodgeImpulse = 18000.0f;

    // --- Advanced Kinetics ---
    [ExportGroup("Advanced Kinetics")]
    [Export] public float RotorInertia = 25.0f;
    [Export] public float UniversalGravitationalConstant = 0.05f;
    [Export] public float TetherStiffness = 150000.0f;
    [Export] public float TetherDamping = 8000.0f;
    [Export] public float TetherRestLength = 5.0f;
    [Export] public int TetherSubSteps = 10;

    // =========================================================================
    // TELEMETRY & DIAGNOSTICS CONFIGURATION
    // =========================================================================
    [ExportGroup("Telemetry & Diagnostics")]
    [Export] public bool EnableTelemetry = true;
    [Export] public float TelemetryPrintRateHz = 10.0f; // High frequency for real-time plotting
    [Export] public string UdpIp = "127.0.0.1";
    [Export] public int UdpPort = 9870; // Dedicated UDP port for RoverDrone
    [Export] public bool EnableAnomalyDetector = true;

    private UdpClient _udpClient;
    private float _telemetryTimer = 0.0f;
    private float _lastUdpErrorTime = -10.0f;
    private const float ERROR_LOG_INTERVAL_SEC = 2.0f;

    // Last computed telemetry cache variables
    private float _lastVertVelPidOut = 0.0f;
    private float _lastPitchPidOut = 0.0f;
    private float _lastRollPidOut = 0.0f;
    private float _lastYawPidOut = 0.0f;

    // --- Core Systems State ---
    private RigidBody3D _chassis = null!;
    private Camera3D _camera = null!;
    private Node3D _camPivot = null!;
    private RichTextLabel _hud = null!;
    private List<GravityWell> _activeGravityWells = new List<GravityWell>();
    private RigidBody3D _gravityAnomaly = null!;
    private bool _spawnGravityAnomaly = false;

    // --- Visual Systems & Telemetry ---
    private MeshInstance3D _gForcePendulum = null!;
    private Vector3 _lastVelocity = Vector3.Zero;
    private float _cameraFovBase = 75.0f;
    private FastNoiseLite _camShakeNoise = new FastNoiseLite();
    private float _shakeTime = 0f;

    // --- Terrain Deformation State ---
    private Vector4[] _craters = new Vector4[16];
    private int _craterIndex = 0;
    private ShaderMaterial _terrainMaterial = null!;

    // --- Relativistic & Environmental State ---
    private Vector3 _gravityDir = Vector3.Down;
    private float _gravityMag = 9.81f;
    private Basis _localReferenceFrame = Basis.Identity;
    private ulong _lastPortalTime = 0;
    private float _ambientFluidDensity = 1.225f;

    // --- Kinetic State ---
    private float _cmgRPM = 0.0f;
    private float _driveRPM = 0.0f;
    private float _transitionTimer = 0.0f;
    private bool _isTethered = false;
    private Vector3 _tetherAnchorGlobal;

    // --- Inputs ---
    private bool _wasTabPressed, _was1, _was2, _was3, _was4, _wasZ, _wasC, _wasF = false;
    private float _camYaw = 0f;
    private float _camPitch = -0.2f;
    private const float MouseSensitivity = 0.003f;
    private bool _isWallRiding = false;

    // --- Flight PIDs ---
    private PIDController _vertVelPID = new PIDController(3000f, 50f, 600f, 15000f);
    private PIDController _pitchPID = new PIDController(4500f, 40f, 1200f, 8000f);
    private PIDController _rollPID = new PIDController(4500f, 40f, 1200f, 8000f);
    private PIDController _yawPID = new PIDController(2500f, 10f, 600f, 4000f);

    private class WheelRotorData
    {
      public Vector3 LocalPosition;
      public bool IsSteerable, IsPowered;

      // Emergent States
      public bool IsIntact = true;
      public bool IsAeroBraking = false;

      // Core Nodes
      public Node3D SteerPivot = null!;
      public Node3D FoldPivot = null!;
      public MeshInstance3D VisualMesh = null!;
      public MeshInstance3D PlumeMesh = null!;
      public GpuParticles3D TractionSmoke = null!;

      public float SpinAngle;
      public float VisualFoldX;
      public float CurrentVisualYOffset;
      public float SmoothPlumeIntensity;

      // Physics State
      public float HitDistance;
      public float SteerAngle;
      public bool IsGrounded;
      public float ActualSlipAngle;

      // Decoupled Kinematics
      public float CmgAngularVelocity;
      public float TireRollVelocity;
      public float CurrentThrust;

      public float SpinDirection = 1.0f;
    }

    private List<WheelRotorData> _nodes = new List<WheelRotorData>();
    private WheelRotorData _fl = null!, _fr = null!, _rl = null!, _rr = null!;

    // --- Shaders ---
    private const string DEFORMABLE_TERRAIN_SHADER = @"
      shader_type spatial;
    varying vec3 v_world_pos;

    uniform vec4 craters[16]; // xyz = pos, w = radius/intensity
    uniform int crater_count;

    void vertex() {
      v_world_pos = (MODEL_MATRIX * vec4(VERTEX, 1.0)).xyz;
      float displacement = 0.0;

      for(int i = 0; i < 16; i++) {
        if (i >= crater_count) break;
        float dist = length(v_world_pos.xz - craters[i].xz);
        float radius = craters[i].w;
        if (dist < radius * 2.0) {
          float influence = smoothstep(radius, 0.0, dist);
          displacement -= influence * radius * 0.4; // Core dip
          displacement += smoothstep(radius * 1.5, radius * 0.7, dist) * influence * radius * 0.15; // Outer rim
        }
      }
      VERTEX.y += displacement;
    }
    void fragment() {
      vec2 grid = abs(fract(v_world_pos.xz * 0.5) - 0.5);
      float line = smoothstep(0.45, 0.5, max(grid.x, grid.y));
      vec3 base_col = vec3(0.02, 0.02, 0.03);
      vec3 grid_col = vec3(0.0, 0.8, 0.5);

      // Color variation based on depth (highlighting craters/grooves)
      float depth_mask = clamp(-v_world_pos.y * 0.5, 0.0, 1.0);
      vec3 damage_col = vec3(0.8, 0.3, 0.0);

      ALBEDO = mix(mix(base_col, grid_col, line), damage_col, depth_mask);
      ROUGHNESS = 0.9 - depth_mask * 0.4;
    }
    ";

    private const string FLUID_PLUME_SHADER = @"
      shader_type spatial;
    render_mode blend_add, unshaded, cull_disabled, depth_draw_never;
    float hash(vec2 p) { return fract(sin(dot(p, vec2(12.9898, 78.233))) * 43758.5453); }
    float noise(vec2 p) {
      vec2 i = floor(p); vec2 f = fract(p); f = f * f * (3.0 - 2.0 * f);
      return mix(mix(hash(i + vec2(0.0, 0.0)), hash(i + vec2(1.0, 0.0)), f.x),
          mix(hash(i + vec2(0.0, 1.0)), hash(i + vec2(1.0, 1.0)), f.x), f.y);
    }
    void vertex() {
      float expansion_factor = pow(UV.y, 2.0);
      float flutter = noise(vec2(VERTEX.y * 20.0 - TIME * 30.0, TIME * 15.0));
      vec3 push_dir = normalize(vec3(NORMAL.x, 0.0, NORMAL.z));
      VERTEX += push_dir * (flutter * 0.2 * expansion_factor);
    }
    void fragment() {
      float vertical_fade = UV.y;
      vec2 uv_scroll1 = vec2(UV.x * 5.0, UV.y * 4.0 - TIME * 14.0);
      vec2 uv_scroll2 = vec2(UV.x * 8.0 + TIME * 2.0, UV.y * 7.0 - TIME * 22.0);
      float n = noise(uv_scroll1) * 0.5 + noise(uv_scroll2) * 0.5;
      float fire_mask = smoothstep(0.3, 0.6, n - (vertical_fade * 0.7));
      float fresnel = max(dot(VIEW, NORMAL), 0.0);
      float edge_fade = smoothstep(0.0, 0.4, fresnel);
      vec3 core_col = vec3(1.0, 0.9, 0.7); vec3 mid_col  = vec3(1.0, 0.4, 0.0); vec3 tip_col  = vec3(0.7, 0.05, 0.0);
      vec3 color = mix(mid_col, core_col, fire_mask);
      color = mix(tip_col, color, 1.0 - pow(vertical_fade, 1.5));
      ALBEDO = color * (1.5 + fire_mask * 4.0);
      ALPHA = fire_mask * edge_fade * (1.0 - pow(vertical_fade, 2.0));
    }
    ";

    private const string MODERN_SMOKE_SHADER = @"
      shader_type spatial;
    // Corrected Render Mode: Force standard alpha blending and explicitly disable depth writing
    render_mode blend_mix, depth_draw_never, cull_back, unshaded;

    uniform sampler2D depth_texture : hint_depth_texture, repeat_disable, filter_nearest;

    void vertex() {
      // Strict spherical billboarding to prevent quad edge visibility
      MODELVIEW_MATRIX = VIEW_MATRIX * mat4(INV_VIEW_MATRIX[0], INV_VIEW_MATRIX[1], INV_VIEW_MATRIX[2], MODEL_MATRIX[3]);
      MODELVIEW_NORMAL_MATRIX = mat3(MODELVIEW_MATRIX);
    }

    void fragment() {
      vec2 uv = UV * 2.0 - 1.0;
      float dist = length(uv);

      // Procedural soft-edge gradient
      float alpha = smoothstep(1.0, 0.2, dist);
      if (alpha < 0.01) discard;

      // Proximity fade: Read depth buffer to prevent harsh clipping lines
      float depth = texture(depth_texture, SCREEN_UV).x;
      vec3 ndc = vec3(SCREEN_UV * 2.0 - 1.0, depth);
      vec4 world = INV_PROJECTION_MATRIX * vec4(ndc, 1.0);
      float depth_z = world.z / world.w;
      float proximity_fade = smoothstep(0.0, 1.5, VERTEX.z - depth_z);

      ALBEDO = COLOR.rgb;
      ALPHA = alpha * COLOR.a * proximity_fade;
    }
    ";

    private Shader _portalShader = null!;

    public override void _Ready()
    {
      Engine.PhysicsTicksPerSecond = 120;
      Input.MouseMode = Input.MouseModeEnum.Captured;
      _camShakeNoise.Frequency = 0.5f;

      CompileShaders();
      SetupEnvironment();
      BuildProceduralTestTrack();
      BuildPlayground();
      BuildChassis();
      InitializeHybridNodes();
      SetupCameraAndHUD();
      InitTelemetrySocket();
    }

    private void InitTelemetrySocket()
    {
      if (!EnableTelemetry) return;

      try
      {
        _udpClient = new UdpClient();
      }
      catch (Exception ex)
      {
        GD.PrintErr($"[TELEMETRY INIT ERROR] Failed to instantiate UdpClient: {ex.Message}");
      }
    }

    public override void _UnhandledInput(InputEvent @event)
    {
      if (@event is InputEventMouseMotion mouseMotion && Input.MouseMode == Input.MouseModeEnum.Captured)
      {
        _camYaw -= mouseMotion.Relative.X * MouseSensitivity;
        _camPitch += mouseMotion.Relative.Y * MouseSensitivity;
        _camPitch = Mathf.Clamp(_camPitch, -Mathf.Pi / 2f + 0.1f, Mathf.Pi / 2f - 0.1f);
      }

      if (@event is InputEventKey keyEvent && keyEvent.Pressed && keyEvent.Keycode == Key.Escape)
        Input.MouseMode = Input.MouseModeEnum.Visible;

      if (@event is InputEventMouseButton mb && mb.Pressed && Input.MouseMode == Input.MouseModeEnum.Visible)
        Input.MouseMode = Input.MouseModeEnum.Captured;
    }

    public override void _ExitTree()
    {
      _udpClient?.Close();
      _udpClient?.Dispose();
      _udpClient = null;
    }

    public override void _PhysicsProcess(double delta)
    {
      float dt = (float)delta;

      // Continuous Centripetal Orbit Manager for Anomaly
      if (_gravityAnomaly != null)
      {
        Vector3 orbitCenter = new Vector3(0, 20, 0);
        Vector3 toCenter = orbitCenter - _gravityAnomaly.GlobalPosition;
        Vector3 tangentialForce = toCenter.Cross(Vector3.Up).Normalized();
        float orbitalSpeed = 800000.0f;
        _gravityAnomaly.ApplyCentralForce((toCenter.Normalized() * 500000.0f) + (tangentialForce * orbitalSpeed));
      }

      UpdateReferenceFrame();
      HandleNBodyAndTopologicalGravity(dt);
      ApplyFluidDynamics(dt);
      UpdatePilotGForces(dt);

      _chassis.ApplyCentralForce(_gravityDir * _gravityMag * _chassis.Mass);

      HandleModeTransition(dt);
      HandleInput(dt);
      HandleTetherMechanics(dt);

      float progress = Mathf.Clamp(_transitionTimer / TransitionDuration, 0f, 1f);

      ApplyGyroscopicPrecession();
      ApplyGyroscopicStabilization(dt, progress);
      ApplyCarPhysics(dt, progress);
      ApplyDronePhysics(dt, progress);

      UpdateVisuals(progress, dt);
      UpdateCamera(dt);
      UpdateHUD();

      if (EnableAnomalyDetector) DetectAnomalies();

      if (EnableTelemetry)
      {
        _telemetryTimer += dt;
        if (TelemetryPrintRateHz > 0 && _telemetryTimer >= (1.0f / TelemetryPrintRateHz))
        {
          LogTelemetry();
          _telemetryTimer = 0.0f;
        }
      }

      _lastVelocity = _chassis.LinearVelocity;
    }

    // =========================================================================
    // REAL-TIME ANOMALY DETECTOR
    // =========================================================================
    private void DetectAnomalies()
    {
      if (_chassis == null) return;

      if (float.IsNaN(_chassis.Position.X) || float.IsInfinity(_chassis.Position.X))
        GD.PrintErr("[ANOMALY] NaN or Infinity detected in Rover Chassis Position!");

      if (float.IsNaN(_chassis.LinearVelocity.X) || float.IsInfinity(_chassis.LinearVelocity.X))
        GD.PrintErr("[ANOMALY] NaN or Infinity detected in Rover Velocity!");

      if (_fl.CurrentThrust > MaxMotorThrust * 1.5f)
        GD.PrintErr($"[ANOMALY] Thruster Over-saturation detected on FL: {_fl.CurrentThrust:F1} N!");

      if (_gravityMag > 200.0f)
        GD.PrintErr($"[ANOMALY] Extreme Gravity Field Gradient detected: {_gravityMag:F1} m/s²!");
    }

    // =========================================================================
    // TELEMETRY LOGGING (TIME-SERIES MAGNITUDE METRICS ONLY)
    // =========================================================================
    private void LogTelemetry()
    {
      if (_udpClient == null)
      {
        ReportTelemetryError("UdpClient is uninitialized or null.");
        return;
      }

      Vector3 accel = (_chassis.LinearVelocity - _lastVelocity) / (1.0f / (float)Engine.PhysicsTicksPerSecond);

      // Kept exclusively for PID error evaluation, excluded from UDP magnitude reporting
      Vector3 localForward = _localReferenceFrame.Inverse() * (-_chassis.GlobalBasis.Z);
      Vector3 localRight = _localReferenceFrame.Inverse() * _chassis.GlobalBasis.X;

      var metrics = new
      {
        timestamp = Time.GetTicksMsec() / 1000.0f,
        transition_progress = Mathf.Clamp(_transitionTimer / TransitionDuration, 0f, 1f),

        // Consolidated Magnitudes (sqrt(x^2 + y^2 + z^2))
        velocity_mag = _chassis.LinearVelocity.Length(),
        ang_rate_mag = _chassis.AngularVelocity.Length(),
        g_force_mag = accel.Length() / 9.81f,

        // Flight Controller Tracking Errors & PID Outputs
        pitch_actual = Mathf.Asin(Mathf.Clamp(localForward.Y, -1f, 1f)),
        roll_actual = Mathf.Asin(Mathf.Clamp(-localRight.Y, -1f, 1f)),
        vert_vel_actual = _chassis.LinearVelocity.Dot(-_gravityDir),

        vert_vel_pid_out = _lastVertVelPidOut,
        pitch_pid_out = _lastPitchPidOut,
        roll_pid_out = _lastRollPidOut,
        yaw_pid_out = _lastYawPidOut,

        // Actuators & Powertrain
        thrust_fl = _fl.CurrentThrust,
        thrust_fr = _fr.CurrentThrust,
        thrust_rl = _rl.CurrentThrust,
        thrust_rr = _rr.CurrentThrust,
        cmg_rpm = _cmgRPM,
        drive_rpm = _driveRPM,

        // Suspension Continuous Loads
        susp_hit_dist_fl = _fl.HitDistance,
        susp_hit_dist_fr = _fr.HitDistance,
        susp_hit_dist_rl = _rl.HitDistance,
        susp_hit_dist_rr = _rr.HitDistance,
        slip_angle_fl = _fl.ActualSlipAngle,
        slip_angle_fr = _fr.ActualSlipAngle,

        // Environmental Topological Dynamics
        gravity_mag = _gravityMag
      };

      try
      {
        string jsonString = JsonSerializer.Serialize(metrics);
        byte[] payload = Encoding.UTF8.GetBytes(jsonString);
        _udpClient.Send(payload, payload.Length, UdpIp, UdpPort);
      }
      catch (SocketException ex)
      {
        ReportTelemetryError($"SocketException on port {UdpPort}: {ex.Message} (Code: {ex.SocketErrorCode})");
      }
      catch (Exception ex)
      {
        ReportTelemetryError($"Unexpected telemetry serialization/transmission error: {ex.Message}");
      }
    }

    private void ReportTelemetryError(string message)
    {
      float currentTime = Time.GetTicksMsec() / 1000.0f;
      if (currentTime - _lastUdpErrorTime >= ERROR_LOG_INTERVAL_SEC)
      {
        GD.PrintErr($"[ROVER TELEMETRY ERROR] {message}");
        _lastUdpErrorTime = currentTime;
      }
    }

    // =============================================================================
    // KINETICS: N-BODY & TOPOLOGICAL GRAVITY
    // =============================================================================
    private void HandleNBodyAndTopologicalGravity(float dt)
    {
      if (_isInZeroGZone)
      {
        _gravityMag = 0f;
        return;
      } else {
        _gravityMag = 9.81f;
      }
      if (_gravityMag < 0.1f) return;

      // 1. BASE GRAVITY
      Vector3 baseGravity = Vector3.Down * 9.81f;
      float structuralSpeed = _chassis.LinearVelocity.Length();

      if (CurrentMode == VehicleMode.Car)
      {
        var spaceState = GetWorld3D().DirectSpaceState;
        Vector3 localForwardVelocity = -_chassis.GlobalBasis.Z * Mathf.Max(0, _chassis.LinearVelocity.Dot(-_chassis.GlobalBasis.Z));
        Vector3 rayOrigin = _chassis.GlobalPosition + (localForwardVelocity * 0.15f);
        float castLength = _isWallRiding ? 5.0f : 2.5f;
        Vector3 rayDir = -_chassis.GlobalBasis.Y * castLength;

        var query = PhysicsRayQueryParameters3D.Create(rayOrigin, rayOrigin + rayDir);
        query.Exclude = new Godot.Collections.Array<Rid> { _chassis.GetRid() };
        var result = spaceState.IntersectRay(query);

        if (result.Count > 0)
        {
          Vector3 hitNormal = (Vector3)result["normal"];
          float alignment = hitNormal.Dot(Vector3.Up);
          float requiredSpeed = _isWallRiding ? 10.0f : 18.0f;

          if (structuralSpeed > requiredSpeed || alignment > 0.8f)
          {
            baseGravity = -hitNormal * 9.81f;
            _isWallRiding = alignment <= 0.8f;
          }
          else _isWallRiding = false;
        }
        else _isWallRiding = false;
      }
      else _isWallRiding = false;

      // 2. N-BODY SUPERPOSITION
      Vector3 netGravity = baseGravity;
      foreach (var well in _activeGravityWells)
      {
        Vector3 r = well.GlobalPosition - _chassis.GlobalPosition;
        float distSq = Mathf.Max(r.LengthSquared(), 1.0f);
        float accelMag = UniversalGravitationalConstant * (well.Mass / distSq);
        netGravity += r.Normalized() * accelMag;
      }

      // N-Body Gravitational Anomaly Pull with Plummer Softening
      if (_gravityAnomaly != null)
      {
        Vector3 toAnomaly = _gravityAnomaly.GlobalPosition - _chassis.GlobalPosition;
        float dist = toAnomaly.Length();

        if (dist < 150.0f)
        {
          float epsilon = 8.0f;
          float rawForceMag = (8000.0f * _chassis.Mass) / (dist * dist + epsilon * epsilon);
          float cappedForceMag = Mathf.Min(rawForceMag, 40000.0f);

          _chassis.ApplyCentralForce(toAnomaly.Normalized() * cappedForceMag);

          if (dist < 50.0f) {
            float warpInfluence = 1.0f - (dist / 50.0f);
            netGravity = netGravity.Lerp(toAnomaly.Normalized() * netGravity.Length(), warpInfluence);
          }
        }
      }

      Vector3 targetGravityDir = netGravity.Normalized();
      float targetGravityMag = netGravity.Length();

      // 3. SAFE TEMPORAL INTEGRATION
      if (targetGravityDir.LengthSquared() < 0.001f) targetGravityDir = Vector3.Down;

      _gravityDir = _gravityDir.Normalized();
      targetGravityDir = targetGravityDir.Normalized();
      float dot = _gravityDir.Dot(targetGravityDir);

      if (dot < -0.999f)
      {
        Vector3 arbitraryAxis = Mathf.Abs(_gravityDir.Z) < 0.999f ? Vector3.Forward : Vector3.Right;
        Vector3 rotAxis = _gravityDir.Cross(arbitraryAxis).Normalized();
        _gravityDir = _gravityDir.Rotated(rotAxis, dt * 8.0f).Normalized();
      }
      else if (dot > 0.999f)
      {
        _gravityDir = targetGravityDir;
      }
      else
      {
        _gravityDir = _gravityDir.Lerp(targetGravityDir, dt * 8.0f).Normalized();
      }

      _gravityMag = Mathf.Lerp(_gravityMag, targetGravityMag, dt * 8.0f);
    }

    // =============================================================================
    // KINETICS: VOLUMETRIC FLUID DYNAMICS (NON-NEWTONIAN DRAG)
    // =============================================================================
    private void ApplyFluidDynamics(float dt)
    {
      _ambientFluidDensity = _chassis.GlobalPosition.Y < -2.0f ? 1000.0f : 1.225f;

      float speedSq = _chassis.LinearVelocity.LengthSquared();
      if (speedSq < 0.1f) return;

      Vector3 velDir = _chassis.LinearVelocity.Normalized();
      float verticalDot = Mathf.Abs(_chassis.GlobalBasis.Y.Dot(velDir));
      float dynamicArea = Mathf.Lerp(TrackWidth * 0.4f, TrackWidth * WheelBase, verticalDot);
      float dragCoefficient = 0.45f;
      float dragForceMag = 0.5f * _ambientFluidDensity * speedSq * dragCoefficient * dynamicArea;

      _chassis.ApplyCentralForce(-velDir * dragForceMag);
    }

    // =============================================================================
    // KINETICS: SUB-STEPPED TENSILE GRAPPLING
    // =============================================================================
    private void HandleTetherMechanics(float dt)
    {
      if (Input.IsPhysicalKeyPressed(Key.F) && !_wasF)
      {
        if (!_isTethered) {
          var spaceState = GetWorld3D().DirectSpaceState;
          var query = PhysicsRayQueryParameters3D.Create(_chassis.GlobalPosition, _chassis.GlobalPosition - _chassis.GlobalBasis.Z * 50.0f);
          var result = spaceState.IntersectRay(query);
          if (result.Count > 0) {
            _tetherAnchorGlobal = (Vector3)result["position"];
            _isTethered = true;
          }
        } else _isTethered = false;
      }
      _wasF = Input.IsPhysicalKeyPressed(Key.F);

      if (!_isTethered) return;

      float subDt = dt / TetherSubSteps;
      Vector3 accumulatedForce = Vector3.Zero;
      Vector3 simulatedPos = _chassis.GlobalPosition;
      Vector3 simulatedVel = _chassis.LinearVelocity;

      for (int i = 0; i < TetherSubSteps; i++)
      {
        Vector3 r = simulatedPos - _tetherAnchorGlobal;
        float currentLength = r.Length();
        if (currentLength > TetherRestLength)
        {
          Vector3 n = r.Normalized();
          float extension = currentLength - TetherRestLength;
          Vector3 springForce = -TetherStiffness * extension * n;
          Vector3 dampForce = -TetherDamping * simulatedVel.Dot(n) * n;
          Vector3 totalForce = springForce + dampForce;

          accumulatedForce += totalForce;
          simulatedVel += (totalForce / Mass) * subDt;
          simulatedPos += simulatedVel * subDt;
        }
      }

      _chassis.ApplyCentralForce(accumulatedForce / TetherSubSteps);
    }

    // =============================================================================
    // KINETICS: GYROSCOPICS & CMG BRAKING
    // =============================================================================
    private void ApplyGyroscopicPrecession()
    {
      Vector3 totalRotorAngularMomentum = Vector3.Zero;

      foreach (var node in _nodes)
      {
        if (!node.IsIntact) continue;
        Vector3 localSpinAxis = _chassis.GlobalBasis.Y;
        float omega = node.CmgAngularVelocity;
        totalRotorAngularMomentum += localSpinAxis * (RotorInertia * omega);
      }

      Vector3 precessionTorque = _chassis.AngularVelocity.Cross(totalRotorAngularMomentum);
      _chassis.ApplyTorque(precessionTorque);
    }

    private void TriggerCMGBrake()
    {
      Vector3 totalLostMomentum = Vector3.Zero;

      foreach (var node in _nodes)
      {
        if (!node.IsIntact) continue;
        float initialOmega = node.CmgAngularVelocity;
        float targetOmega = initialOmega * 0.1f;
        float deltaOmega = initialOmega - targetOmega;
        Vector3 deltaL = _chassis.GlobalBasis.Y * (RotorInertia * deltaOmega);

        totalLostMomentum += deltaL;
        node.CmgAngularVelocity = targetOmega;
      }

      _cmgRPM *= 0.1f;
      _chassis.ApplyTorqueImpulse(totalLostMomentum);
    }

    private void UpdatePilotGForces(float dt)
    {
      if (_chassis == null || _gForcePendulum == null) return;

      Vector3 accel = (_chassis.LinearVelocity - _lastVelocity) / dt;
      Vector3 localAccel = _chassis.GlobalBasis.Inverse() * accel;

      Vector3 targetPos = new Vector3(-localAccel.X, -localAccel.Y, -localAccel.Z) * 0.015f;
      targetPos = new Vector3(
          Mathf.Clamp(targetPos.X, -0.4f, 0.4f),
          Mathf.Clamp(targetPos.Y, -0.4f, 0.4f),
          Mathf.Clamp(targetPos.Z, -0.4f, 0.4f)
          );

      _gForcePendulum.Position = _gForcePendulum.Position.Lerp(targetPos + new Vector3(0, 0.2f, 0), dt * 10f);
    }

    // =============================================================================
    // ENVIRONMENT & REFERENCE FRAMES
    // =============================================================================
    private void UpdateReferenceFrame()
    {
      Vector3 localUp = -_gravityDir;
      Vector3 arbitraryAxis = Mathf.Abs(localUp.Dot(Vector3.Right)) > 0.9f ? Vector3.Forward : Vector3.Right;
      Vector3 localRight = arbitraryAxis.Cross(localUp).Normalized();
      Vector3 localForward = localUp.Cross(localRight).Normalized();
      _localReferenceFrame = new Basis(localRight, localUp, localForward);
    }

    private void TeleportEntity(RigidBody3D body, Area3D inPortal, Area3D outPortal)
    {
      Transform3D inTrans = inPortal.GlobalTransform;
      Transform3D outTrans = outPortal.GlobalTransform;
      Transform3D flipY = new Transform3D(new Basis(Vector3.Up, Mathf.Pi), Vector3.Zero);

      Transform3D relativeTrans = inTrans.AffineInverse() * body.GlobalTransform;
      Transform3D newTrans = outTrans * flipY * relativeTrans;

      newTrans.Origin += -outPortal.GlobalBasis.Z * 2.0f;
      PhysicsServer3D.BodySetState(body.GetRid(), PhysicsServer3D.BodyState.Transform, newTrans);

      // Inside TeleportEntity():
      Basis portalRotation = outTrans.Basis * flipY.Basis * inTrans.Basis.Inverse();

      _gravityDir = (portalRotation * _gravityDir).Normalized();
      _localReferenceFrame = portalRotation * _localReferenceFrame; // Keeps frame continuous

      Vector3 newLinVel = portalRotation * body.LinearVelocity;
      Vector3 newAngVel = portalRotation * body.AngularVelocity;

      PhysicsServer3D.BodySetState(body.GetRid(), PhysicsServer3D.BodyState.LinearVelocity, newLinVel);
      PhysicsServer3D.BodySetState(body.GetRid(), PhysicsServer3D.BodyState.AngularVelocity, newAngVel);

      ResetPIDs();
    }

    private bool _isInZeroGZone = false;
    private void RegisterZeroGZone(Area3D zone)
    {
      zone.BodyEntered += (Node3D body) => { if (body == _chassis) _isInZeroGZone = true; };
      zone.BodyExited  += (Node3D body) => { if (body == _chassis) _isInZeroGZone = false; };
    }

    // =============================================================================
    // CORE KINEMATICS & ABILITIES
    // =============================================================================
    private void HandleModeTransition(float dt)
    {
      bool isTabPressed = Input.IsPhysicalKeyPressed(Key.Tab);
      if (isTabPressed && !_wasTabPressed)
      {
        if (CurrentMode == VehicleMode.Car || CurrentMode == VehicleMode.TransitioningToCar)
          CurrentMode = VehicleMode.TransitioningToDrone;
        else if (CurrentMode == VehicleMode.Drone || CurrentMode == VehicleMode.TransitioningToDrone)
          CurrentMode = VehicleMode.TransitioningToCar;
        ResetPIDs();
      }
      _wasTabPressed = isTabPressed;

      if (CurrentMode == VehicleMode.TransitioningToDrone)
      {
        _transitionTimer = Mathf.Min(_transitionTimer + dt, TransitionDuration);
        if (_transitionTimer >= TransitionDuration) CurrentMode = VehicleMode.Drone;
      }
      else if (CurrentMode == VehicleMode.TransitioningToCar)
      {
        _transitionTimer = Mathf.Max(_transitionTimer - dt, 0);
        if (_transitionTimer <= 0) CurrentMode = VehicleMode.Car;
      }
    }

    private void HandleInput(float dt)
    {
      if (Input.IsPhysicalKeyPressed(Key.Key1) && !_was1) _fl.IsIntact = !_fl.IsIntact;
      if (Input.IsPhysicalKeyPressed(Key.Key2) && !_was2) _fr.IsIntact = !_fr.IsIntact;
      if (Input.IsPhysicalKeyPressed(Key.Key3) && !_was3) _rl.IsIntact = !_rl.IsIntact;
      if (Input.IsPhysicalKeyPressed(Key.Key4) && !_was4) _rr.IsIntact = !_rr.IsIntact;

      _was1 = Input.IsPhysicalKeyPressed(Key.Key1); _was2 = Input.IsPhysicalKeyPressed(Key.Key2);
      _was3 = Input.IsPhysicalKeyPressed(Key.Key3); _was4 = Input.IsPhysicalKeyPressed(Key.Key4);

      if (Input.IsPhysicalKeyPressed(Key.B)) TriggerCMGBrake();

      if (CurrentMode == VehicleMode.Drone || _transitionTimer > TransitionDuration / 2f)
      {
        if (Input.IsPhysicalKeyPressed(Key.Z) && !_wasZ) _chassis.ApplyCentralImpulse(-_chassis.GlobalBasis.X * DodgeImpulse);
        if (Input.IsPhysicalKeyPressed(Key.C) && !_wasC) _chassis.ApplyCentralImpulse(_chassis.GlobalBasis.X * DodgeImpulse);
      }
      _wasZ = Input.IsPhysicalKeyPressed(Key.Z); _wasC = Input.IsPhysicalKeyPressed(Key.C);

      float targetSteer = (Input.IsPhysicalKeyPressed(Key.A) ? 1f : (Input.IsPhysicalKeyPressed(Key.D) ? -1f : 0f)) * MaxSteerAngle;
      _fl.SteerAngle = Mathf.Lerp(_fl.SteerAngle, targetSteer, dt * 10f);
      _fr.SteerAngle = Mathf.Lerp(_fr.SteerAngle, targetSteer, dt * 10f);

      bool revving = Input.IsPhysicalKeyPressed(Key.W);

      if (CurrentMode == VehicleMode.Car)
      {
        float maxWheelAngVel = 0;
        foreach (var w in _nodes) maxWheelAngVel = Mathf.Max(maxWheelAngVel, Mathf.Abs(w.TireRollVelocity));
        if (revving) _driveRPM = Mathf.Lerp(_driveRPM, 6000f, dt * 0.8f);
        else _driveRPM = Mathf.Lerp(_driveRPM, maxWheelAngVel * 9.55f, dt * 2.0f);

        _cmgRPM = Mathf.Lerp(_cmgRPM, 0f, dt * 0.5f);
      }
      else
      {
        if (revving) _cmgRPM = Mathf.Lerp(_cmgRPM, 8000f, dt * 1.5f);
        else _cmgRPM = Mathf.Lerp(_cmgRPM, 1500f, dt * 0.5f);
        _driveRPM = Mathf.Lerp(_driveRPM, 0f, dt * 2.0f);
      }

      foreach(var n in _nodes)
      {
        if(n.IsIntact) n.CmgAngularVelocity = _cmgRPM * 0.1047f * n.SpinDirection;
      }

      if (Input.IsPhysicalKeyPressed(Key.R))
      {
        _chassis.GlobalPosition = new Vector3(0, 5, 0);
        _chassis.LinearVelocity = _chassis.AngularVelocity = Vector3.Zero;
        _chassis.Rotation = Vector3.Zero;
        _gravityDir = Vector3.Down; _gravityMag = 9.81f;
        _isTethered = false;
        ResetPIDs();
      }
    }

    private void ResetPIDs()
    {
      _vertVelPID.Reset(); _pitchPID.Reset(); _rollPID.Reset(); _yawPID.Reset();
    }

    private void ApplyGyroscopicStabilization(float dt, float progress)
    {
      if (CurrentMode != VehicleMode.Car)
      {
        _chassis.AngularDamp = Mathf.Lerp(0.8f, 3.5f, progress);
        _chassis.LinearDamp = Mathf.Lerp(0.15f, 1.0f, progress);

        if (CurrentMode == VehicleMode.TransitioningToDrone)
        {
          Vector3 currentUp = _chassis.GlobalBasis.Y;
          Vector3 dynamicUp = -_gravityDir;
          Vector3 restorativeAxis = currentUp.Cross(dynamicUp).Normalized();
          float angleToUpright = Mathf.Acos(Mathf.Clamp(currentUp.Dot(dynamicUp), -1f, 1f));

          if (angleToUpright > 0.05f) _chassis.ApplyTorque(restorativeAxis * angleToUpright * 8000.0f);
        }
      }
      else
      {
        _chassis.AngularDamp = 0.8f;
        _chassis.LinearDamp = 0.15f;
      }
    }

    private void ApplyCarPhysics(float dt, float progress)
    {
      if (progress > 0.8f) return;

      var spaceState = GetWorld3D().DirectSpaceState;
      float groundWeight = 1.0f - progress;
      float currentRestLength = RestLength * groundWeight;
      bool inAir = true;

      float forwardSpeed = _chassis.LinearVelocity.Dot(-_chassis.GlobalBasis.Z);
      float accelInput = Input.IsPhysicalKeyPressed(Key.W) ? 1f : (Input.IsPhysicalKeyPressed(Key.S) ? -1f : 0f);

      bool isAeroBraking = CurrentMode == VehicleMode.Car && accelInput < -0.1f && forwardSpeed > 8.0f;
      bool isDamaged = !_fl.IsIntact || !_fr.IsIntact || !_rl.IsIntact || !_rr.IsIntact;

      foreach (var node in _nodes)
      {
        node.IsAeroBraking = isAeroBraking;

        if (!node.IsIntact)
        {
          node.IsGrounded = false;
          node.HitDistance = currentRestLength + SpringTravel;
          node.TractionSmoke.Emitting = false;
          continue;
        }

        float safetyMargin = 0.5f;
        Vector3 rayOrigin = _chassis.ToGlobal(node.LocalPosition) + _chassis.GlobalBasis.Y * safetyMargin;
        Vector3 rayDir = -_chassis.GlobalBasis.Y;
        float castLength = currentRestLength + SpringTravel + safetyMargin;

        var query = new PhysicsShapeQueryParameters3D {
          Shape = new SphereShape3D { Radius = WheelRadius * 0.8f },
          Transform = new Transform3D(Basis.Identity, rayOrigin),
          Motion = rayDir * castLength
        };
        query.Exclude = new Godot.Collections.Array<Rid> { _chassis.GetRid() };

        float[] castResult = spaceState.CastMotion(query);
        float safeFraction = castResult[0];
        float rawHitDistance = (safeFraction * castLength) - safetyMargin;

        node.IsGrounded = rawHitDistance < currentRestLength + SpringTravel;
        node.HitDistance = rawHitDistance;

        if (node.IsGrounded)
        {
          inAir = false;
          float compression = currentRestLength - node.HitDistance;

          if (compression > 0)
          {
            Vector3 strutTopOffset = _chassis.ToGlobal(node.LocalPosition) - _chassis.GlobalPosition;
            Vector3 pointVel = _chassis.LinearVelocity + _chassis.AngularVelocity.Cross(strutTopOffset);
            float suspensionVelocity = -pointVel.Dot(_chassis.GlobalBasis.Y);

            // Non-Linear Progressive Spring Integrator
            float springForce = (compression * BaseSpringStiffness) + (Mathf.Pow(compression, 3.0f) * ProgressiveStiffness);
            float dampForce = suspensionVelocity * SpringDamping;
            float suspensionForce = Mathf.Max(0, (springForce + dampForce) * groundWeight);

            float maxForceAllowed = Mass * 9.81f * 15.0f;
            suspensionForce = Mathf.Clamp(suspensionForce, 0, maxForceAllowed);

            _chassis.ApplyForce(_chassis.GlobalBasis.Y * suspensionForce, strutTopOffset);

            if (suspensionForce > 35000.0f) RegisterCrater(_chassis.ToGlobal(node.LocalPosition) - _chassis.GlobalBasis.Y * node.HitDistance, suspensionForce);

            if (CurrentMode == VehicleMode.Car || CurrentMode == VehicleMode.TransitioningToCar)
            {
              ProcessTireGrip(node, pointVel, dt, groundWeight, accelInput);
            }
          }
          else node.TractionSmoke.Emitting = false;
        }
        else node.TractionSmoke.Emitting = false;
      }

      if (isAeroBraking)
      {
        foreach (var n in _nodes)
        {
          if (n.IsIntact) {
            n.CurrentThrust = MaxMotorThrust * 0.8f;
            _chassis.ApplyForce(_chassis.GlobalBasis.Z * n.CurrentThrust, _chassis.GlobalBasis * n.LocalPosition);
          } else n.CurrentThrust = 0f;
        }
      }
      else if (isDamaged && CurrentMode == VehicleMode.Car)
      {
        Vector3 localForward = _chassis.GlobalBasis.Inverse() * (-_chassis.GlobalBasis.Z);
        Vector3 localRight = _chassis.GlobalBasis.Inverse() * _chassis.GlobalBasis.X;
        Vector3 localAngVel = _chassis.GlobalBasis.Inverse() * _chassis.AngularVelocity;

        float pCmd = _pitchPID.Update(0 - Mathf.Asin(Mathf.Clamp(localForward.Y, -1f, 1f)), localAngVel.X, dt);
        float rCmd = _rollPID.Update(0 - Mathf.Asin(Mathf.Clamp(localRight.Y, -1f, 1f)), localAngVel.Z, dt);

        _fl.CurrentThrust = _fl.IsIntact ? Mathf.Max(0, pCmd + rCmd) : 0f;
        _fr.CurrentThrust = _fr.IsIntact ? Mathf.Max(0, pCmd - rCmd) : 0f;
        _rl.CurrentThrust = _rl.IsIntact ? Mathf.Max(0, -pCmd + rCmd) : 0f;
        _rr.CurrentThrust = _rr.IsIntact ? Mathf.Max(0, -pCmd - rCmd) : 0f;

        foreach (var n in _nodes) {
          if (n.IsIntact && n.CurrentThrust > 0) _chassis.ApplyForce(_chassis.GlobalBasis.Y * n.CurrentThrust, _chassis.GlobalBasis * n.LocalPosition);
        }
      }
      else
      {
        foreach (var n in _nodes) n.CurrentThrust = 0f;
      }

      if (CurrentMode == VehicleMode.Car && Input.IsPhysicalKeyPressed(Key.Space))
      {
        bool isUpsideDown = _chassis.GlobalBasis.Y.Dot(-_gravityDir) < -0.2f;

        if (!inAir && !isUpsideDown)
        {
          // Standard upright jump
          _chassis.ApplyCentralImpulse(_chassis.GlobalBasis.Y * Mass * 0.7f);
        }
        else if (isUpsideDown)
        {
          // Cockroach Hop: Punch downwards from the roof relative to local -Y
          // to launch the inverted chassis off the ground with a slight rotational kick
          Vector3 hopImpulse = -_chassis.GlobalBasis.Y * (Mass * 0.3f);
          Vector3 torqueKick = _chassis.GlobalBasis.Z * (Mass * 0.05f);

          _chassis.ApplyCentralImpulse(hopImpulse);
          _chassis.ApplyTorqueImpulse(torqueKick);
        }
      }

      if (CurrentMode == VehicleMode.TransitioningToDrone && forwardSpeed > 2.0f)
      {
        _chassis.ApplyCentralForce(_chassis.GlobalBasis.Y * (forwardSpeed * 150.0f * progress));
      }
    }

    private void ProcessTireGrip(WheelRotorData wheel, Vector3 pointVel, float dt, float weight, float accelInput)
    {
      Vector3 wheelBasisForward = -_chassis.GlobalBasis.Z;
      Vector3 wheelBasisRight = _chassis.GlobalBasis.X;

      if (wheel.IsSteerable)
      {
        wheelBasisForward = wheelBasisForward.Rotated(_chassis.GlobalBasis.Y, wheel.SteerAngle);
        wheelBasisRight = wheelBasisRight.Rotated(_chassis.GlobalBasis.Y, wheel.SteerAngle);
      }

      float forwardSpeed = pointVel.Dot(wheelBasisForward);
      if(CurrentMode == VehicleMode.Car) wheel.TireRollVelocity = forwardSpeed / WheelRadius;

      float lateralVelocity = pointVel.Dot(wheelBasisRight);
      if (Mathf.Abs(lateralVelocity) < 0.05f) lateralVelocity = 0f;

      float targetSlipAngle = Mathf.Abs(forwardSpeed) > 0.1f ? Mathf.Atan2(lateralVelocity, Mathf.Abs(forwardSpeed)) : 0f;
      wheel.ActualSlipAngle = Mathf.Lerp(wheel.ActualSlipAngle, targetSlipAngle, 10f * dt);

      // Traction Volumetrics
      wheel.TractionSmoke.Emitting = Mathf.Abs(wheel.ActualSlipAngle) > 0.35f || (wheel.IsPowered && Mathf.Abs(accelInput) > 0.8f && forwardSpeed < 5.0f);

      float lateralForceMag = CalculatePacejka(wheel.ActualSlipAngle, 1.5f, 1.3f, GripFriction * Mass * 0.25f, 0.3f);
      lateralForceMag *= Mathf.Clamp(pointVel.Length() / 1.0f, 0f, 1f) * weight;

      Vector3 appPoint = _chassis.ToGlobal(wheel.LocalPosition) - _chassis.GlobalBasis.Y * wheel.HitDistance - _chassis.GlobalPosition;
      _chassis.ApplyForce(-wheelBasisRight * lateralForceMag, appPoint);

      if (wheel.IsPowered)
      {
        _chassis.ApplyForce(wheelBasisForward * accelInput * (EngineForce * 0.25f) * weight, appPoint);
      }
    }

    private float CalculatePacejka(float slipAngle, float b, float c, float d, float e)
    {
      return d * Mathf.Sin(c * Mathf.Atan(b * slipAngle - e * (b * slipAngle - Mathf.Atan(b * slipAngle))));
    }

    private void RegisterCrater(Vector3 worldPos, float force)
    {
      if (_terrainMaterial == null) return;
      float radius = Mathf.Clamp(force / 20000.0f, 1.0f, 6.0f);
      _craters[_craterIndex] = new Vector4(worldPos.X, worldPos.Y, worldPos.Z, radius);
      _craterIndex = (_craterIndex + 1) % 16;

      _terrainMaterial.SetShaderParameter("craters", _craters);
      _terrainMaterial.SetShaderParameter("crater_count", Mathf.Min(_craterIndex + 1, 16));
    }

    private void ApplyDronePhysics(float dt, float progress)
    {
      if (progress <= 0.01f) return;

      Vector3 localForward = _localReferenceFrame.Inverse() * (-_chassis.GlobalBasis.Z);
      Vector3 localRight = _localReferenceFrame.Inverse() * _chassis.GlobalBasis.X;
      Vector3 localAngVel = _chassis.GlobalBasis.Inverse() * _chassis.AngularVelocity;

      float currentPitch = Mathf.Asin(Mathf.Clamp(localForward.Y, -1f, 1f));
      float currentRoll = Mathf.Asin(Mathf.Clamp(-localRight.Y, -1f, 1f));

      float targetPitch = Input.IsPhysicalKeyPressed(Key.W) ? -1.4f : (Input.IsPhysicalKeyPressed(Key.S) ? 1.4f : 0f);
      float targetRoll = Input.IsPhysicalKeyPressed(Key.A) ? -0.9f : (Input.IsPhysicalKeyPressed(Key.D) ? 0.9f : 0f);
      float yawInput = Input.IsPhysicalKeyPressed(Key.Q) ? 8f : (Input.IsPhysicalKeyPressed(Key.E) ? -8f : 0f);
      float targetVertVel = Input.IsPhysicalKeyPressed(Key.Space) ? 10.0f : (Input.IsPhysicalKeyPressed(Key.Shift) ? -20.0f : 0f);

      var currSpaceState = GetWorld3D().DirectSpaceState;
      var query = PhysicsRayQueryParameters3D.Create(_chassis.GlobalPosition, _chassis.GlobalPosition + _gravityDir * 50.0f);
      var result = currSpaceState.IntersectRay(query);

      float groundEffectMultiplier = 1.0f;
      if (result.Count > 0)
      {
        float dist = _chassis.GlobalPosition.DistanceTo((Vector3)result["position"]);
        if (dist < 2.5f) groundEffectMultiplier += 1.2f * Mathf.Exp(-dist * 1.5f);
      }

      float flightPowerMultiplier = Mathf.Clamp(_cmgRPM / 5000f, 0.3f, 1.2f);
      float dynamicMaxThrust = MaxMotorThrust * flightPowerMultiplier;

      // Evaluate PIDs ONCE and store the result directly into the telemetry cache variables
      _lastVertVelPidOut = _vertVelPID.Update(targetVertVel - _chassis.LinearVelocity.Dot(-_gravityDir), 0f, dt);
      float baseThrust = ((Mass * _gravityMag) / (4.0f * groundEffectMultiplier)) + _lastVertVelPidOut;

      _lastPitchPidOut = _pitchPID.Update(targetPitch - currentPitch, localAngVel.X, dt) * progress;
      _lastRollPidOut  = _rollPID.Update(targetRoll - currentRoll, -localAngVel.Z, dt) * progress;
      _lastYawPidOut   = _yawPID.Update(yawInput - localAngVel.Y, 0, dt) * progress;

      // Use the cached variables to mix the motor thrusts
      _fl.CurrentThrust = _fl.IsIntact ? Mathf.Clamp(baseThrust + _lastPitchPidOut + _lastRollPidOut + _lastYawPidOut, 0, dynamicMaxThrust) : 0;
      _fr.CurrentThrust = _fr.IsIntact ? Mathf.Clamp(baseThrust + _lastPitchPidOut - _lastRollPidOut - _lastYawPidOut, 0, dynamicMaxThrust) : 0;
      _rl.CurrentThrust = _rl.IsIntact ? Mathf.Clamp(baseThrust - _lastPitchPidOut + _lastRollPidOut - _lastYawPidOut, 0, dynamicMaxThrust) : 0;
      _rr.CurrentThrust = _rr.IsIntact ? Mathf.Clamp(baseThrust - _lastPitchPidOut - _lastRollPidOut + _lastYawPidOut, 0, dynamicMaxThrust) : 0;

      foreach (var node in _nodes)
      {
        if (node.IsIntact && node.CurrentThrust > 0) {
          _chassis.ApplyForce(_chassis.GlobalBasis.Y * node.CurrentThrust * progress, _chassis.GlobalBasis * node.LocalPosition);
        }
      }

      _chassis.ApplyTorque(_chassis.GlobalBasis * new Vector3(0, (_fl.CurrentThrust + _rr.CurrentThrust - _fr.CurrentThrust - _rl.CurrentThrust) * YawDragFactor, 0) * progress);
    }

    // =============================================================================
    // VISUALS & LEVEL ARCHITECTURE
    // =============================================================================
    private void UpdateVisuals(float progress, float dt)
    {
      float easeProgress = progress * progress * (3f - 2f * progress);
      float expectedHoverThrust = (Mass * 9.81f) / 4.0f;

      foreach (var node in _nodes)
      {
        if (!node.IsIntact) {
          node.VisualMesh.Visible = node.PlumeMesh.Visible = false;
          continue;
        }
        node.VisualMesh.Visible = true;

        node.CurrentVisualYOffset = Mathf.Lerp(node.CurrentVisualYOffset, node.IsGrounded ? node.HitDistance : RestLength + SpringTravel, dt * 30.0f);
        float visualYOffset = Mathf.Lerp(node.CurrentVisualYOffset, 0f, easeProgress);

        node.SteerPivot.Position = node.LocalPosition - new Vector3(0, visualYOffset, 0);
        node.SteerPivot.Rotation = new Vector3(0, node.SteerAngle, 0);

        node.VisualFoldX = Mathf.Lerp(node.VisualFoldX, node.IsAeroBraking ? Mathf.Pi / 2f : 0f, dt * 15f);
        node.FoldPivot.Rotation = new Vector3(node.VisualFoldX, 0, Mathf.Lerp(0f, Mathf.Pi / 2f, easeProgress) * -Mathf.Sign(node.LocalPosition.X));

        float visualSpin = CurrentMode == VehicleMode.Car ? node.TireRollVelocity : node.CmgAngularVelocity;
        node.SpinAngle += (-visualSpin * dt) + ((node.CurrentThrust * 0.005f) * ((_nodes.IndexOf(node) == 0 || _nodes.IndexOf(node) == 3) ? -1 : 1) * dt);

        node.VisualMesh.Transform = new Transform3D(Basis.Identity.Rotated(Vector3.Forward, Mathf.Pi / 2f), Vector3.Zero);
        node.VisualMesh.RotateObjectLocal(Vector3.Up, node.SpinAngle);

        float targetIntensity = Mathf.Clamp(expectedHoverThrust > 0 ? node.CurrentThrust / expectedHoverThrust : 0, 0f, 2.5f);
        if (CurrentMode != VehicleMode.Car && !node.IsAeroBraking) targetIntensity *= progress;
        if (CurrentMode == VehicleMode.Car && !node.IsAeroBraking) targetIntensity = 0f;

        node.SmoothPlumeIntensity = Mathf.Lerp(node.SmoothPlumeIntensity, targetIntensity, dt * 15.0f);

        if (node.SmoothPlumeIntensity > 0.05f)
        {
          node.PlumeMesh.Visible = true;

          Vector3 exhaustDir = node.IsAeroBraking ? _chassis.GlobalBasis.Z : -_chassis.GlobalBasis.Y;
          Vector3 anchorOffset = node.IsAeroBraking ? new Vector3(0, 0, WheelRadius) : new Vector3(0, -WheelRadius, 0);
          Vector3 globalAnchor = node.SteerPivot.GlobalPosition + _chassis.GlobalBasis * anchorOffset;

          Vector3 upRef = Mathf.Abs(exhaustDir.Dot(Vector3.Up)) > 0.99f ? Vector3.Right : Vector3.Up;
          Vector3 rightDir = upRef.Cross(exhaustDir).Normalized();
          node.PlumeMesh.GlobalBasis = new Basis(rightDir, exhaustDir, exhaustDir.Cross(rightDir).Normalized());

          float scaleY = node.SmoothPlumeIntensity * 1.5f;
          float scaleXZ = Mathf.Min(1.2f, scaleY);

          node.PlumeMesh.Scale = new Vector3(scaleXZ, scaleY, scaleXZ);
          node.PlumeMesh.GlobalPosition = globalAnchor + exhaustDir * (scaleY * 0.5f);
        }
        else node.PlumeMesh.Visible = false;
      }
    }

    private void InitializeHybridNodes()
    {
      _fl = CreateHybridNode(new Vector3(-TrackWidth / 2f, 0, -WheelBase / 2f), true, true);
      _fr = CreateHybridNode(new Vector3(TrackWidth / 2f, 0, -WheelBase / 2f), true, true);
      _rl = CreateHybridNode(new Vector3(-TrackWidth / 2f, 0, WheelBase / 2f), false, true);
      _rr = CreateHybridNode(new Vector3(TrackWidth / 2f, 0, WheelBase / 2f), false, true);

      _fl.SpinDirection = 1.0f; _fr.SpinDirection = -1.0f;
      _rl.SpinDirection = -1.0f; _rr.SpinDirection = 1.0f;

      _nodes.AddRange(new[] { _fl, _fr, _rl, _rr });
    }

    private WheelRotorData CreateHybridNode(Vector3 localPos, bool isSteerable, bool isPowered)
    {
      var steerPivot = new Node3D();
      var foldPivot = new Node3D();

      var visual = new MeshInstance3D {
        Mesh = new CylinderMesh { Height = 0.2f, TopRadius = WheelRadius, BottomRadius = WheelRadius },
        MaterialOverride = new StandardMaterial3D { AlbedoColor = Colors.DarkOrange }
      };

      var plume = new MeshInstance3D {
        Mesh = new CylinderMesh { TopRadius = WheelRadius * 0.4f, BottomRadius = 0.01f, Height = 1.2f },
        MaterialOverride = new ShaderMaterial { Shader = new Shader { Code = FLUID_PLUME_SHADER } },
        Visible = false
      };

      var smokeMat = new ParticleProcessMaterial {
        EmissionShape = ParticleProcessMaterial.EmissionShapeEnum.Sphere,
        EmissionSphereRadius = 0.2f, Direction = new Vector3(0, 1, 0), Spread = 45f,
        InitialVelocityMin = 2f, InitialVelocityMax = 6f, Gravity = new Vector3(0, 2, 0),
        Color = new Color(0.8f, 0.8f, 0.8f, 0.5f)
      };
      var smokeMesh = new QuadMesh { Size = new Vector2(1.2f, 1.2f) };
      smokeMesh.Material = new ShaderMaterial { Shader = new Shader { Code = MODERN_SMOKE_SHADER } };

      var smokeSys = new GpuParticles3D {
        ProcessMaterial = smokeMat, DrawPass1 = smokeMesh, Emitting = false,
        Amount = 45, Lifetime = 1.2f
      };

      _chassis.AddChild(steerPivot);
      steerPivot.AddChild(foldPivot);
      foldPivot.AddChild(visual);
      foldPivot.AddChild(smokeSys);
      AddChild(plume);

      smokeSys.Position = new Vector3(0, -WheelRadius, 0);

      return new WheelRotorData {
        LocalPosition = localPos, IsSteerable = isSteerable, IsPowered = isPowered,
        SteerPivot = steerPivot, FoldPivot = foldPivot, VisualMesh = visual,
        PlumeMesh = plume, TractionSmoke = smokeSys, SpinAngle = 0f, SmoothPlumeIntensity = 0f
      };
    }

    private void BuildChassis()
    {
      _chassis = new RigidBody3D { Mass = Mass, LinearDamp = 0.15f, AngularDamp = 0.8f, GravityScale = 0f };
      _chassis.Position = new Vector3(0, 5, 0);

      // Cache dimensions to keep the math clean
      float bodyWidth = TrackWidth * 0.8f;
      float bodyHeight = 0.5f;
      float bodyLength = WheelBase + 1.0f;

      // 1. Main Hull (White Space-Grade Material)
      var hull = new MeshInstance3D {
        Mesh = new BoxMesh { Size = new Vector3(bodyWidth, bodyHeight, bodyLength) },
        MaterialOverride = new StandardMaterial3D { AlbedoColor = Colors.WhiteSmoke }
      };
      _chassis.AddChild(hull);
      _chassis.AddChild(new CollisionShape3D { Shape = new BoxShape3D { Size = new Vector3(bodyWidth, bodyHeight, bodyLength) } });

      // 2. Front Cockpit / Camera Array (-Z is forward)
      var cockpit = new MeshInstance3D {
        Mesh = new BoxMesh { Size = new Vector3(bodyWidth * 0.6f, 0.4f, 0.8f) },
        MaterialOverride = new StandardMaterial3D {
          AlbedoColor = Colors.Black,
          Roughness = 0.1f // Gives it a glossy, glass-like look
        }
      };
      // Placed on top, pushed to the front
      cockpit.Position = new Vector3(0, bodyHeight / 2 + 0.2f, -bodyLength / 2.0f + 0.4f);
      _chassis.AddChild(cockpit);

      // 3. Rear Solar Panel / Cargo Bed (+Z is backward)
      var solarPanel = new MeshInstance3D {
        Mesh = new BoxMesh { Size = new Vector3(bodyWidth * 0.8f, 0.05f, bodyLength * 0.4f) },
        MaterialOverride = new StandardMaterial3D {
          AlbedoColor = Colors.DarkSlateBlue,
          Metallic = 0.8f
        }
      };
      // Placed on top, pushed to the rear
      solarPanel.Position = new Vector3(0, bodyHeight / 2 + 0.025f, bodyLength / 4.0f);
      _chassis.AddChild(solarPanel);

      // 4. Front Headlights (Glowing Cyan)
      var frontLightsMat = new StandardMaterial3D {
        AlbedoColor = Colors.LightCyan,
        EmissionEnabled = true,
        Emission = Colors.LightCyan,
        EmissionEnergyMultiplier = 4.0f
      };
      for (int i = -1; i <= 1; i += 2) // Generates left (i=-1) and right (i=1)
      {
        var headlight = new MeshInstance3D {
          Mesh = new BoxMesh { Size = new Vector3(0.2f, 0.15f, 0.1f) },
          MaterialOverride = frontLightsMat,
          Position = new Vector3(i * (bodyWidth / 2.0f - 0.2f), 0, -bodyLength / 2.0f - 0.05f)
        };
        _chassis.AddChild(headlight);
      }

      // 5. Rear Taillights (Glowing Red)
      var rearLightsMat = new StandardMaterial3D {
        AlbedoColor = Colors.Red,
        EmissionEnabled = true,
        Emission = Colors.Red,
        EmissionEnergyMultiplier = 3.0f
      };
      for (int i = -1; i <= 1; i += 2)
      {
        var taillight = new MeshInstance3D {
          Mesh = new BoxMesh { Size = new Vector3(0.3f, 0.1f, 0.1f) },
          MaterialOverride = rearLightsMat,
          Position = new Vector3(i * (bodyWidth / 2.0f - 0.2f), 0, bodyLength / 2.0f + 0.05f)
        };
        _chassis.AddChild(taillight);
      }

      // 6. G-Force Pendulum (Pilot Mass)
      _gForcePendulum = new MeshInstance3D {
        Mesh = new SphereMesh { Radius = 0.2f },
        MaterialOverride = new StandardMaterial3D {
          AlbedoColor = Colors.Crimson,
          EmissionEnabled = true,
          Emission = Colors.Red,
          EmissionEnergyMultiplier = 2.0f
        }
      };
      // Center it in the cabin
      _gForcePendulum.Position = new Vector3(0, 0.2f, 0);
      _chassis.AddChild(_gForcePendulum);

      AddChild(_chassis);
    }

    private void CompileShaders()
    {
      _portalShader = new Shader { Code = @"
        shader_type spatial;
        render_mode unshaded, cull_disabled;
        uniform vec4 portal_color : source_color = vec4(0.0, 0.5, 1.0, 1.0);
        void fragment() {
          vec2 uv = UV * 2.0 - 1.0;
          float dist = length(uv);
          if (dist > 1.0) discard;
          float ripple = sin(dist * 30.0 - TIME * 15.0) * 0.5 + 0.5;
          float rim = smoothstep(0.8, 1.0, dist);
          ALBEDO = portal_color.rgb * (ripple * 0.5 + rim * 2.0);
          ALPHA = 1.0;
        }"
      };

      _terrainMaterial = new ShaderMaterial { Shader = new Shader { Code = DEFORMABLE_TERRAIN_SHADER } };
    }

    private void BuildProceduralTestTrack()
    {
      var terrainNode = new StaticBody3D();
      var planeMesh = new PlaneMesh { Size = new Vector2(400, 400), SubdivideWidth = 150, SubdivideDepth = 150 };

      var st = new SurfaceTool();
      st.CreateFrom(planeMesh, 0);
      var arrayMesh = st.Commit();

      var mdt = new MeshDataTool();
      mdt.CreateFromSurface(arrayMesh, 0);
      var noise = new FastNoiseLite { Frequency = 0.05f, FractalType = FastNoiseLite.FractalTypeEnum.Fbm };

      for (int i = 0; i < mdt.GetVertexCount(); i++)
      {
        Vector3 v = mdt.GetVertex(i);
        float dist = new Vector2(v.X, v.Z).Length();
        float mask = Mathf.Clamp((dist - 10.0f) / 20.0f, 0.0f, 1.0f);
        v.Y = noise.GetNoise2D(v.X, v.Z) * 4.0f * mask;
        mdt.SetVertex(i, v);
      }

      arrayMesh.ClearSurfaces();
      mdt.CommitToSurface(arrayMesh);
      st.Begin(Mesh.PrimitiveType.Triangles);
      st.CreateFrom(arrayMesh, 0);
      st.GenerateNormals();
      var finalMesh = st.Commit();

      terrainNode.AddChild(new MeshInstance3D { Mesh = finalMesh, MaterialOverride = _terrainMaterial });
      terrainNode.AddChild(new CollisionShape3D { Shape = finalMesh.CreateTrimeshShape() });

      terrainNode.Position = new Vector3(0, -1.5f, 0);
      AddChild(terrainNode);
    }

    private void BuildPlayground()
    {
      // 1. Floor (10x larger: 1000x1000 units)
      // Placed at Y = -1 with height 2, so the top surface sits flush at Y = 0
      var floor = new StaticBody3D { Position = new Vector3(0, -1, 0) };
      floor.AddChild(new MeshInstance3D { Mesh = new BoxMesh { Size = new Vector3(1000, 2, 1000) }, MaterialOverride = _terrainMaterial });
      floor.AddChild(new CollisionShape3D { Shape = new BoxShape3D { Size = new Vector3(1000, 2, 1000) } });
      AddChild(floor);

      // 2. Skatepark Quarterpipe / Launch Ramp (Connects flush to ground at Y = 0)
      var rampNode = new StaticBody3D { Position = new Vector3(0, 0, -100) };

      float rampWidth = 40.0f;
      float rampLength = 180.0f;
      float rampHeight = 60.0f;

      // Generate curved mesh and trimesh collider for smooth transitions
      Mesh rampMesh = CreateSkateRampMesh(rampWidth, rampLength, rampHeight, segments: 16);
      rampNode.AddChild(new MeshInstance3D { Mesh = rampMesh, MaterialOverride = _terrainMaterial });
      rampNode.AddChild(new CollisionShape3D { Shape = rampMesh.CreateTrimeshShape() });
      AddChild(rampNode);

      // 3. Portals
      var portalIn = CreatePortal(new Vector3(0, 90, -350), Vector3.Back, Colors.Cyan);
      var portalOut = CreatePortal(new Vector3(0, 100, 50), Vector3.Down, Colors.Orange);

      // Bi-directional connections
      portalIn.BodyEntered += (Node3D body) => {
        if (body is RigidBody3D rb && rb == _chassis) {
          ulong now = Time.GetTicksMsec();
          if (now - _lastPortalTime >= 200) { _lastPortalTime = now; TeleportEntity(rb, portalIn, portalOut); }
        }
      };
      portalOut.BodyEntered += (Node3D body) => {
        if (body is RigidBody3D rb && rb == _chassis) {
          ulong now = Time.GetTicksMsec();
          if (now - _lastPortalTime >= 200) { _lastPortalTime = now; TeleportEntity(rb, portalOut, portalIn); }
        }
      };

      // 4. Wall Segments
      for (int i = 0; i < 5; i++)
      {
        var wallSeg = new StaticBody3D { Position = new Vector3(-30 - i * 2, i * 4, -50), RotationDegrees = new Vector3(0, 0, -(i + 1) * 15.0f) };
        wallSeg.AddChild(new MeshInstance3D { Mesh = new BoxMesh { Size = new Vector3(10, 2, 100) }, MaterialOverride = _terrainMaterial });
        wallSeg.AddChild(new CollisionShape3D { Shape = new BoxShape3D { Size = new Vector3(10, 2, 100) } });
        AddChild(wallSeg);
      }

      // 5. Zero-G Zone
      var zeroGZone = new Area3D { Position = new Vector3(0, 20, 50) };
      zeroGZone.AddChild(new CollisionShape3D { Shape = new BoxShape3D { Size = new Vector3(80, 400, 80) } });

      var zoneMat = new StandardMaterial3D
      {
        AlbedoColor = new Color(0, 1, 0, 0.1f),
        Transparency = BaseMaterial3D.TransparencyEnum.Alpha,
        CullMode = BaseMaterial3D.CullModeEnum.Disabled // Renders inner faces when camera is inside
      };

      zeroGZone.AddChild(new MeshInstance3D
          {
          Mesh = new BoxMesh { Size = new Vector3(80, 400, 80) },
          MaterialOverride = zoneMat
          });

      AddChild(zeroGZone);
      RegisterZeroGZone(zeroGZone);

      // 6. Gravity Well
      var well = new GravityWell { Position = new Vector3(0, 150, -200) };
      AddChild(well);
      _activeGravityWells.Add(well);

      // 7. Gravitational Anomaly
      if (_spawnGravityAnomaly) {
        _gravityAnomaly = new RigidBody3D { Mass = 1000000f, GravityScale = 0f, Position = new Vector3(50, 20, 50) };
        _gravityAnomaly.AddChild(new MeshInstance3D { Mesh = new SphereMesh { Radius = 4.0f }, MaterialOverride = new StandardMaterial3D { AlbedoColor = Colors.Purple, EmissionEnabled = true, Emission = Colors.DarkViolet } });
        _gravityAnomaly.AddChild(new CollisionShape3D { Shape = new SphereShape3D { Radius = 4.0f } });
        AddChild(_gravityAnomaly);
        _gravityAnomaly.Position = new Vector3(80, 20, 0);
        _gravityAnomaly.LinearDamp = 0.0f;
        _gravityAnomaly.AngularVelocity = new Vector3(0, 2.5f, 0);
      }
    }

    /// <summary>
    /// Helper to construct a smooth curved quarterpipe mesh using SurfaceTool.
    /// </summary>
    private ArrayMesh CreateSkateRampMesh(float width, float length, float height, int segments)
    {
      var st = new SurfaceTool();
      st.Begin(Mesh.PrimitiveType.Triangles);

      float halfWidth = width * 0.5f;

      for (int i = 0; i <= segments; i++)
      {
        float t = (float)i / segments;
        float z = -t * length;

        // Quadratic curve for smooth ramp transition: ground-level flush entry at t=0
        float y = height * Mathf.Pow(t, 2);

        // Calculate surface normal along curve tangent
        Vector3 tangent = new Vector3(0, 2 * height * t / length, -1).Normalized();
        Vector3 normal = tangent.Cross(Vector3.Right).Normalized();

        st.SetNormal(normal);
        st.SetUV(new Vector2(0, t));
        st.AddVertex(new Vector3(-halfWidth, y, z));

        st.SetNormal(normal);
        st.SetUV(new Vector2(1, t));
        st.AddVertex(new Vector3(halfWidth, y, z));
      }

      // Build quad indices
      for (int i = 0; i < segments; i++)
      {
        int row1 = i * 2;
        int row2 = (i + 1) * 2;

        st.AddIndex(row1);
        st.AddIndex(row2);
        st.AddIndex(row1 + 1);

        st.AddIndex(row1 + 1);
        st.AddIndex(row2);
        st.AddIndex(row2 + 1);
      }

      return st.Commit();
    }

    private Area3D CreatePortal(Vector3 pos, Vector3 forward, Color color)
    {
      var portalNode = new Area3D();
      AddChild(portalNode);
      portalNode.GlobalPosition = pos;

      Vector3 up = Math.Abs(forward.Dot(Vector3.Up)) > 0.99f ? Vector3.Right : Vector3.Up;
      portalNode.GlobalBasis = Basis.LookingAt(forward, up);

      var mat = new ShaderMaterial { Shader = _portalShader };
      mat.SetShaderParameter("portal_color", color);

      portalNode.AddChild(new MeshInstance3D { Mesh = new PlaneMesh { Size = new Vector2(8.0f, 8.0f) }, MaterialOverride = mat, RotationDegrees = new Vector3(90, 0, 0) });
      portalNode.AddChild(new CollisionShape3D { Shape = new CylinderShape3D { Radius = 4.0f, Height = 0.5f }, RotationDegrees = new Vector3(90, 0, 0) });
      return portalNode;
    }

    private void SetupEnvironment()
    {
      AddChild(new WorldEnvironment { Environment = new Godot.Environment { BackgroundMode = Godot.Environment.BGMode.Color, BackgroundColor = new Color(0.1f, 0.1f, 0.15f), AmbientLightSource = Godot.Environment.AmbientSource.Color, AmbientLightColor = new Color(0.4f, 0.4f, 0.4f) } });
      AddChild(new DirectionalLight3D { ShadowEnabled = true, Position = new Vector3(10, 50, 10), RotationDegrees = new Vector3(-45, 45, 0) });
    }

    private void SetupCameraAndHUD()
    {
      _camPivot = new Node3D(); AddChild(_camPivot);
      _camera = new Camera3D { Current = true, Fov = _cameraFovBase }; _camPivot.AddChild(_camera);
      _camera.Position = new Vector3(0, 4, 12); _camera.LookAt(Vector3.Zero);

      var canvas = new CanvasLayer(); AddChild(canvas);

      // Initialize as RichTextLabel for BBCode support
      _hud = new RichTextLabel
      {
        Position = new Vector2(20, 20),
        Size = new Vector2(800, 400), // RichTextLabel needs an explicit bounding box
        BbcodeEnabled = true,
        ScrollActive = false,
        ClipContents = false
      };

      // Apply theme overrides to match your previous LabelSettings
      _hud.AddThemeFontSizeOverride("normal_font_size", 18);
      _hud.AddThemeColorOverride("default_color", Colors.White);
      _hud.AddThemeColorOverride("font_outline_color", Colors.Black);
      _hud.AddThemeConstantOverride("outline_size", 4);

      canvas.AddChild(_hud);
    }

    private void UpdateCamera(float dt)
    {
      if (_chassis == null || _camPivot == null) return;

      float speed = _chassis.LinearVelocity.Length();

      // 1. Kinetic FOV Warping
      _camera.Fov = Mathf.Lerp(_camera.Fov, _cameraFovBase + Mathf.Clamp(speed * 0.4f, 0f, 30f), dt * 5.0f);

      // 2. Aero-Braking Shake
      float accelInput = Input.IsPhysicalKeyPressed(Key.W) ? 1f : (Input.IsPhysicalKeyPressed(Key.S) ? -1f : 0f);
      if (CurrentMode == VehicleMode.Car && accelInput < -0.1f && speed > 8.0f) {
        _shakeTime += dt * 50.0f;
        float shakeX = _camShakeNoise.GetNoise1D(_shakeTime) * 0.05f;
        float shakeY = _camShakeNoise.GetNoise1D(_shakeTime + 100f) * 0.05f;
        _camera.HOffset = Mathf.Lerp(_camera.HOffset, shakeX, dt * 20f);
        _camera.VOffset = Mathf.Lerp(_camera.VOffset, shakeY, dt * 20f);
      } else {
        _camera.HOffset = Mathf.Lerp(_camera.HOffset, 0f, dt * 10f);
        _camera.VOffset = Mathf.Lerp(_camera.VOffset, 0f, dt * 10f);
      }

      // 3. Start with our dynamic local environment frame
      Basis camBasis = _localReferenceFrame;

      // 4. Apply mouse rotations relative to this frame
      camBasis = camBasis.Rotated(camBasis.Y, _camYaw);
      camBasis = camBasis.Rotated(camBasis.X, _camPitch);

      // 5. Calculate target position
      float followDistance = CurrentMode == VehicleMode.Drone ? 8.0f : 6.0f;
      Vector3 offset = camBasis * new Vector3(0, 10.0f, followDistance);
      Vector3 targetPos = _chassis.GlobalPosition + offset;

      _camPivot.GlobalPosition = _camPivot.GlobalPosition.Lerp(targetPos, dt * 15.0f);
      _camera.GlobalPosition = _camPivot.GlobalPosition;

      _camera.LookAt(_chassis.GlobalPosition + _localReferenceFrame.Y * 1.0f, _localReferenceFrame.Y);
    }

    // =========================================================================
    // VISUAL DIAGNOSTICS (DISCRETE METRICS ONLY)
    // =========================================================================
    private string GetDamageTag(bool isIntact) => isIntact ? "[color=green]OK[/color]" : "[color=red]DMG[/color]";
    private string GetContactTag(bool isGrounded) => isGrounded ? "[color=green]CON[/color]" : "[color=red]AIR[/color]";

    // =========================================================================
    // VISUAL DIAGNOSTICS (DISCRETE METRICS ONLY)
    // =========================================================================
    private void UpdateHUD()
    {
      // Damage state logic is inverted: Intact = OK (Green), Not Intact = DMG (Red)
      string damageState = $"FL:{GetDamageTag(_fl.IsIntact)} FR:{GetDamageTag(_fr.IsIntact)} RL:{GetDamageTag(_rl.IsIntact)} RR:{GetDamageTag(_rr.IsIntact)}";
      string groundState = $"FL:{GetContactTag(_fl.IsGrounded)} FR:{GetContactTag(_fr.IsGrounded)} RL:{GetContactTag(_rl.IsGrounded)} RR:{GetContactTag(_rr.IsGrounded)}";

      // Requires _hud to be a RichTextLabel with BbcodeEnabled = true
      _hud.Text = $"SYSTEM MODE: {CurrentMode}\n" +
        $"STATUS: Tether={_isTethered} | WallRiding={_isWallRiding}\n" +
        $"DAMAGE: {damageState}\n" +
        $"CONTACT: {groundState}\n\n" +
        $"[TAB] Mode | [F] Fire Tether | [B] CMG Brake | [1-4] Damage";
    }

  }
}
