# FRC Robot Project Context

## Project Architecture & Stack
- **Language:** Java (JDK 21)
- **Framework:** WPILib Command-Based (Subsystems, Commands, Triggers)
- **Architecture:** AdvantageKit logging framework

## Documentation & References
- **Local Java API Docs:** file:///C:/Users/Public/wpilib/2026/documentation/java/index.html
- **Local FRC Docs:** file:///C:/Users/Public/wpilib/2026/documentation/frc-docs/index.html

## Code Generation Guidelines
- Always use modern Command-Based syntax (`CommandPtr`, `Trigger.onTrue()`, functional commands).
- Prioritize inspectable vendor JSONs in `vendordeps/` for exact library dependencies (e.g., CTRE Phoenix 6, REVLib).
- Write clean, non-blocking asynchronous logic for subsystem periodic loops.