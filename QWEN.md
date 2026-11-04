# FRC Robot Project

## Stack
- Language: Java (version is set by GradleRIO; check `build.gradle` before using newer language features)
- Framework: WPILib 2026, Command-Based (Subsystems, Commands, Triggers)
- Vendor libraries: see `vendordeps/*.json` for the exact libraries and versions. All motors and sensors are CTRE (Phoenix 6: TalonFX, CANcoder, Pigeon 2, CANdle etc.). There are NO REV devices: never suggest REVLib, SparkMax, SparkFlex, or REV APIs.
- Autonomous: BLine (path following library). Do NOT use or suggest PathPlanner or Choreo. Check how BLine is added (`build.gradle` / `vendordeps/`) and its docs for the exact API; never guess method names.
- Logging: WPILib built-ins only (`DataLogManager`, `DriverStation.startDataLog`, NetworkTables, `SmartDashboard`/`Shuffleboard`). Do NOT use or suggest AdvantageKit.
- Build/deploy: GradleRIO. Build with `./gradlew build`, simulate with `./gradlew simulateJava`, deploy with `./gradlew deploy`.

## Project layout
- `src/main/java/frc/robot/` - robot code (`Robot`, `RobotContainer`, `Constants`)
  - `subsystems/` - one class per mechanism
  - `commands/` - command factories and standalone commands
- `src/main/java/frc/lib/` - reusable helpers not tied to this robot
- `vendordeps/` - vendor library JSONs
- `src/main/deploy/` - files copied to the roboRIO (autos, paths, configs)

## Drivetrain and autonomous
- Drivetrain is swerve, all CTRE: TalonFX drive and steer motors, CANcoders, Pigeon 2. Use Phoenix 6 only (`com.ctre.phoenix6`), never Phoenix 5 APIs.
- Prefer CTRE's swerve tooling (Tuner X generated `TunerConstants`, `SwerveDrivetrain`/`CommandSwerveDrivetrain`, `SwerveRequest`s) over hand-rolled swerve modules. Extend the existing generated code instead of rewriting it, and don't hand-edit `TunerConstants` unless I ask.
- Use field-centric control by default, with a clear way to reset the heading. Apply deadbands and slew/acceleration limiting to driver input.
- Odometry/pose: use the drivetrain's pose estimator. Add vision measurements with timestamps if vision is present. Keep one source of truth for robot pose.
- Autos are built with BLine. Auto routines are command compositions that follow BLine paths and run mechanism commands between them. Keep paths in `src/main/deploy/` as the library expects. Make sure autos handle alliance flipping correctly according to BLine's docs.

## Documentation
- Local Java API: file:///C:/Users/Public/wpilib/2026/documentation/java/index.html
- Local FRC docs: file:///C:/Users/Public/wpilib/2026/documentation/frc-docs/index.html
- Online: https://docs.wpilib.org and https://github.wpilib.org/allwpilib/docs/release/java/
- Vendor docs: CTRE Phoenix 6 https://v6.docs.ctr-electronics.com
- BLine: use its official docs/repo for API details (check how it is installed in this project first).
- If unsure about an API, check the docs or the vendor JSON version instead of guessing. Never invent method names.

## Code style
- Use modern Command-Based: command factories on subsystems that return `Command` (e.g. `public Command runIntake()` using `run`, `runOnce`, `startEnd`, `runEnd`), composed with `Commands.sequence/parallel/race/deadline`.
- Bind with `Trigger` / `CommandXboxController`: `controller.a().onTrue(...)`, `.whileTrue(...)`, `.toggleOnTrue(...)`. Set default commands with `setDefaultCommand`.
- Use the WPILib units library (`Units`, `Measure`/typed units) for distances, angles, velocities and voltages instead of bare doubles where practical.
- Constants go in `Constants.java`, grouped in nested classes per subsystem, with units in names or types (e.g. `kMaxSpeedMetersPerSec`).
- Subsystems own their hardware. Hardware objects are `private final`. Expose behavior through commands and small getters, not raw motor access.
- Prefer composition and small subsystems over one giant class.

## Real-time rules (important on the roboRIO)
- Never block: no `Thread.sleep`, no waiting loops, no blocking I/O or network calls in `periodic()`, `execute()`, or constructors that run at enable.
- Keep `periodic()` light. Avoid allocating objects every loop, avoid heavy string formatting, and avoid reflection or large collections in hot paths.
- The main loop runs at 20 ms. If you see "loop overrun" warnings, profile before optimizing, and suggest moving work out of `periodic()` or lowering how often it runs.
- Use CAN status signal refresh sparingly; batch Phoenix 6 signals with `BaseStatusSignal.refreshAll` and set update frequencies deliberately.
- Run slow work in a separate `Notifier` or thread only when really needed, and guard shared state.

## Safety and correctness
- Set current limits, soft limits and neutral modes on every motor controller.
- Always have a safe default state: mechanisms stop when a command ends or the robot is disabled.
- Use `SubsystemBase.addRequirements` / `Subsystem.run*` factories so commands correctly claim their subsystem.
- Handle null/absent values: `DriverStation.getAlliance()` returns an `Optional`.
- Mirror field geometry for the red alliance explicitly, and don't assume blue.

## Testing and verification
- After changes, run `./gradlew build` and fix compile errors and warnings before reporting done.
- Prefer simulation (`simulateJava`) and unit tests (`./gradlew test`) for logic that doesn't need hardware.
- Do not claim code works on the real robot; say what was verified (compiled, simulated, tested) and what still needs hardware testing.

## How to work with me
- Read the existing code and match its style before adding new patterns.
- Make the smallest change that solves the problem; don't refactor unrelated files.
- Do not edit `build.gradle`, `vendordeps/`, or deploy settings unless I ask.
- If a request is ambiguous (e.g. which motor controller, CAN ID, or units), ask instead of guessing.

# right now you are on a diffrent git branch. this is a test and i will see what i will add the the real code and what not