# Requirements Document: Aerodynamics Library Testing

## Introduction

This document specifies comprehensive testing requirements for the aero_simpi C++ library, which performs aerodynamic calculations for rockets and aircraft. The library calculates aerodynamic forces, moments, and coefficients for multi-component vehicles (body, wings, fins) across subsonic, transonic, supersonic, and hypersonic flight regimes. Testing must ensure mathematical correctness, physical validity, and robustness across all operating conditions.

## Glossary

- **System**: The aero_simpi aerodynamics library
- **Parser**: The JSON configuration parser (JsonParser class)
- **Component**: An aerodynamic component (Body, Wing, or Fin)
- **Model**: The complete aerodynamics model (AerodynamicsModel class)
- **State**: Flight state parameters (velocity, angles, atmospheric conditions)
- **Output**: Calculated aerodynamic coefficients and derivatives
- **Coefficient**: Dimensionless aerodynamic force or moment (Cx, Cy, Cz, mx, my, mz)
- **Derivative**: Rate of change of coefficient with respect to state variable
- **Mach_Number**: Ratio of flow velocity to speed of sound (M)
- **Reynolds_Number**: Dimensionless parameter characterizing flow regime (Re)
- **Angle_of_Attack**: Angle between velocity vector and body axis (alpha, degrees)
- **Sideslip_Angle**: Lateral angle between velocity vector and body axis (beta, degrees)
- **Center_of_Pressure**: Point where aerodynamic forces act (x_cp)
- **Static_Margin**: Stability measure (x_cp - x_com) / c_ref
- **Interference**: Aerodynamic interaction between components
- **Stall**: Flow separation causing loss of lift
- **Transonic**: Flight regime near Mach 1 (0.8 < M < 1.2)
- **Catch2**: C++ testing framework used for all tests
- **Property_Test**: Test verifying universal properties across many generated inputs
- **Round_Trip**: Operation followed by its inverse returning to original value

## Requirements

### Requirement 1: JSON Configuration Parsing

**User Story:** As a developer, I want to parse JSON configuration files, so that I can load aerodynamic models from external data.

#### Acceptance Criteria

1. WHEN a valid JSON configuration file is provided, THE Parser SHALL parse it into an AeroConfig object
2. WHEN a JSON file contains all required fields, THE Parser SHALL populate all configuration parameters correctly
3. WHEN a JSON file is missing required fields, THE Parser SHALL throw a ConfigError with a descriptive message
4. WHEN a JSON file contains invalid field types, THE Parser SHALL throw a ConfigError
5. WHEN a JSON file contains invalid component types, THE Parser SHALL throw a ConfigError
6. WHEN a JSON file contains invalid nose types, THE Parser SHALL throw a ConfigError
7. WHEN a JSON file path does not exist, THE Parser SHALL throw a ConfigError
8. WHEN a JSON file contains malformed JSON syntax, THE Parser SHALL throw a ConfigError
9. WHEN parsing succeeds, THE Parser SHALL validate the resulting configuration
10. THE System SHALL support round-trip serialization (parse then serialize produces equivalent JSON)

### Requirement 2: Configuration Validation

**User Story:** As a developer, I want configuration validation, so that I can detect invalid parameters before calculations.

#### Acceptance Criteria

1. WHEN S_ref is zero or negative, THE System SHALL throw a ConfigError
2. WHEN c_ref is zero or negative, THE System SHALL throw a ConfigError
3. WHEN b_ref is zero or negative, THE System SHALL throw a ConfigError
4. WHEN a configuration has no components, THE System SHALL throw a ConfigError
5. WHEN a BODY component has zero or negative length, THE System SHALL throw a ConfigError
6. WHEN a BODY component has zero or negative diameter, THE System SHALL throw a ConfigError
7. WHEN a WING component has zero or negative S_ref, THE System SHALL throw a ConfigError
8. WHEN a FIN component has zero or negative S_ref, THE System SHALL throw a ConfigError
9. WHEN all configuration parameters are valid, THE System SHALL accept the configuration without errors

### Requirement 3: State Validation

**User Story:** As a developer, I want flight state validation, so that I can detect physically impossible conditions.

#### Acceptance Criteria

1. WHEN velocity V is zero or negative, THE System SHALL throw a SingularError
2. WHEN density rho is zero or negative, THE System SHALL throw a SingularError
3. WHEN Mach number is negative or greater than 5.0, THE System SHALL throw a RangeError
4. WHEN angle of attack exceeds ±90 degrees, THE System SHALL throw a RangeError
5. WHEN sideslip angle exceeds ±90 degrees, THE System SHALL throw a RangeError
6. WHEN all state parameters are within valid ranges, THE System SHALL accept the state without errors

### Requirement 4: Component Factory Pattern

**User Story:** As a developer, I want a component factory, so that I can create components polymorphically.

#### Acceptance Criteria

1. WHEN creating a BODY component, THE ComponentFactory SHALL return a BodyComponent instance
2. WHEN creating a WING component, THE ComponentFactory SHALL return a WingComponent instance
3. WHEN creating a FIN component, THE ComponentFactory SHALL return a FinComponent instance
4. WHEN a component configuration has type BODY, THE BodyComponent constructor SHALL accept it
5. WHEN a component configuration has type WING, THE WingComponent constructor SHALL accept it
6. WHEN a component configuration has type FIN, THE FinComponent constructor SHALL accept it
7. WHEN a BodyComponent receives a non-BODY configuration, THE System SHALL throw a ConfigError
8. WHEN a WingComponent receives a non-WING configuration, THE System SHALL throw a ConfigError
9. WHEN a FinComponent receives a non-FIN configuration, THE System SHALL throw a ConfigError

### Requirement 5: Body Component Aerodynamics

**User Story:** As a developer, I want accurate body aerodynamics, so that I can model fuselage forces and moments.

#### Acceptance Criteria

1. WHEN calculating body drag at zero angle of attack, THE System SHALL return positive Cx
2. WHEN angle of attack increases, THE Body SHALL produce increasing normal force magnitude
3. WHEN Mach number transitions from subsonic to supersonic, THE Body SHALL show increased wave drag
4. WHEN angle of attack is zero, THE Body SHALL produce zero normal force (Cz = 0)
5. WHEN angle of attack is positive, THE Body SHALL produce negative Cz (Z-UP convention)
6. WHEN sideslip angle is zero, THE Body SHALL produce zero side force (Cy = 0)
7. WHEN sideslip angle is non-zero, THE Body SHALL produce side force proportional to beta
8. WHEN Mach number exceeds 3.0, THE Body SHALL apply hypersonic corrections
9. WHEN angle of attack exceeds 30 degrees, THE Body SHALL set is_body_stalled flag to true
10. THE Body SHALL calculate center of pressure that moves with Mach number and angle of attack

### Requirement 6: Wing Component Aerodynamics

**User Story:** As a developer, I want accurate wing aerodynamics, so that I can model lifting surfaces.

#### Acceptance Criteria

1. WHEN angle of attack is zero, THE Wing SHALL produce near-zero lift coefficient
2. WHEN angle of attack increases linearly in unstalled regime, THE Wing SHALL produce linearly increasing lift
3. WHEN angle of attack exceeds stall angle, THE Wing SHALL reduce lift coefficient
4. WHEN wing is stalled, THE System SHALL set is_wing_stalled flag to true
5. WHEN Mach number is subsonic, THE Wing SHALL apply Prandtl-Glauert compressibility correction
6. WHEN Mach number is supersonic, THE Wing SHALL apply supersonic lift theory
7. WHEN Mach number is transonic (0.8 < M < 1.2), THE Wing SHALL smoothly interpolate between subsonic and supersonic
8. WHEN Mach number exceeds 3.0, THE Wing SHALL reduce lift effectiveness
9. WHEN angle of attack exceeds 15 degrees, THE Wing SHALL add vortex lift contribution
10. THE Wing SHALL calculate induced drag proportional to lift squared

### Requirement 7: Fin Component Aerodynamics

**User Story:** As a developer, I want accurate fin aerodynamics, so that I can model control surfaces and stabilizers.

#### Acceptance Criteria

1. WHEN fin deflection angle is zero, THE Fin SHALL produce forces based only on angle of attack
2. WHEN fin deflection angle is non-zero, THE Fin SHALL produce additional control forces
3. WHEN fin has mount_angle of 0 degrees, THE Fin SHALL produce forces primarily in Z direction
4. WHEN fin has mount_angle of 90 degrees, THE Fin SHALL produce forces primarily in Y direction
5. WHEN downwash from upstream wing is present, THE Fin SHALL experience reduced effective angle of attack
6. WHEN Mach number is transonic, THE Fin SHALL show reduced control effectiveness
7. WHEN angle of attack exceeds stall angle, THE Fin SHALL reduce effectiveness
8. THE Fin SHALL calculate moments about center of mass based on position
9. THE Fin SHALL apply hysteresis to angle of attack for stall modeling
10. THE Fin SHALL scale forces by fin area relative to reference area

### Requirement 8: Mach Number Weight Calculations

**User Story:** As a developer, I want smooth Mach regime transitions, so that I can avoid discontinuities in calculations.

#### Acceptance Criteria

1. WHEN Mach number is below 0.85, THE System SHALL assign high subsonic weight
2. WHEN Mach number is above 0.95, THE System SHALL assign high supersonic weight
3. WHEN Mach number is above 3.0, THE System SHALL assign high hypersonic weight
4. WHEN Mach number is in transition zones, THE System SHALL use sigmoid functions for smooth interpolation
5. THE System SHALL ensure Mach weights are continuous functions of Mach number
6. THE System SHALL ensure Mach weights are differentiable at all points

### Requirement 9: Component Interference Effects

**User Story:** As a developer, I want interference modeling, so that I can capture wing-body and wake interactions.

#### Acceptance Criteria

1. WHEN a wing and body are present, THE System SHALL apply wing-fuselage interference factor
2. WHEN wing-fuselage interference is applied, THE System SHALL increase wing lift coefficient
3. WHEN a wing and fin are present, THE System SHALL apply wake shadow effect on fin
4. WHEN wake shadow is applied, THE System SHALL reduce fin effectiveness based on wing lift
5. WHEN wing lift coefficient is high, THE System SHALL increase wake shadow effect
6. THE System SHALL ensure wake shadow factor does not reduce dynamic pressure below 50%
7. WHEN no wing is present, THE System SHALL not apply wake shadow to fins
8. WHEN no body is present, THE System SHALL not apply wing-fuselage interference

### Requirement 10: Model Integration and Summation

**User Story:** As a developer, I want correct force summation, so that I can get total vehicle aerodynamics.

#### Acceptance Criteria

1. WHEN calculating total forces, THE Model SHALL sum contributions from all components
2. WHEN calculating total moments, THE Model SHALL sum contributions from all components
3. WHEN calculating center of pressure, THE Model SHALL compute weighted average based on normal forces
4. WHEN total normal force is near zero, THE Model SHALL use geometric average for center of pressure
5. WHEN calculating static margin, THE Model SHALL use (x_cp - x_com) / c_ref formula
6. THE Model SHALL apply interference effects before summation
7. THE Model SHALL set is_transonic flag when 0.8 <= M <= 1.2
8. THE Model SHALL propagate stall flags from any component to output

### Requirement 11: Static Derivatives

**User Story:** As a developer, I want accurate static derivatives, so that I can perform stability analysis.

#### Acceptance Criteria

1. THE System SHALL calculate dCx_dalpha for all components
2. THE System SHALL calculate dCz_dalpha for all components
3. THE System SHALL calculate dmy_dalpha for all components
4. THE System SHALL calculate dCy_dbeta for all components
5. THE System SHALL calculate dmz_dbeta for all components
6. WHEN fin can deflect, THE System SHALL calculate dCz_ddelta
7. WHEN fin can deflect, THE System SHALL calculate dmy_ddelta
8. THE System SHALL express all static derivatives in units of 1/radian
9. THE System SHALL ensure derivatives are continuous functions of state
10. THE System SHALL sum derivatives from all components for total vehicle derivatives

### Requirement 12: Dynamic Derivatives

**User Story:** As a developer, I want accurate dynamic derivatives, so that I can model damping and dynamic stability.

#### Acceptance Criteria

1. THE System SHALL calculate dCz_dq (pitch damping) for all components
2. THE System SHALL calculate dmy_dq (pitch damping moment) for all components
3. THE System SHALL calculate dCy_dr (yaw damping) for all components
4. THE System SHALL calculate dmz_dr (yaw damping moment) for all components
5. THE System SHALL calculate dmx_dp (roll damping) for wing components
6. THE System SHALL express all dynamic derivatives in units of 1/(rad/s)
7. THE System SHALL scale dynamic derivatives by moment arm and velocity
8. THE System SHALL ensure dynamic derivatives remain finite for all valid velocities

### Requirement 13: Physical Bounds and Constraints

**User Story:** As a developer, I want physically realistic outputs, so that I can trust the calculations.

#### Acceptance Criteria

1. THE System SHALL ensure drag coefficient Cx is always positive
2. THE System SHALL ensure lift-to-drag ratio remains within physically realistic bounds
3. THE System SHALL ensure center of pressure remains within vehicle length
4. THE System SHALL ensure static margin remains within reasonable bounds (-2 to +5)
5. THE System SHALL ensure stall factors remain between 0.4 and 1.0
6. THE System SHALL ensure Mach weights remain between 0.0 and 1.0
7. THE System SHALL ensure interference factors remain positive
8. THE System SHALL ensure Reynolds number corrections remain bounded

### Requirement 14: Symmetry Properties

**User Story:** As a developer, I want symmetric behavior, so that I can verify physical correctness.

#### Acceptance Criteria

1. WHEN angle of attack is negated, THE System SHALL negate normal force Cz
2. WHEN angle of attack is negated, THE System SHALL negate pitching moment my
3. WHEN sideslip angle is negated, THE System SHALL negate side force Cy
4. WHEN sideslip angle is negated, THE System SHALL negate yawing moment mz
5. WHEN angle of attack is zero and sideslip is zero, THE System SHALL produce zero side force
6. WHEN angle of attack is zero and sideslip is zero, THE System SHALL produce zero normal force
7. WHEN fin deflection is negated, THE System SHALL negate control forces proportionally
8. THE System SHALL maintain symmetry for symmetric vehicle configurations

### Requirement 15: Continuity and Smoothness

**User Story:** As a developer, I want smooth outputs, so that I can use the model in numerical integration.

#### Acceptance Criteria

1. THE System SHALL produce continuous coefficients across all Mach regimes
2. THE System SHALL produce continuous coefficients across angle of attack range
3. THE System SHALL use sigmoid functions for smooth transitions at regime boundaries
4. THE System SHALL apply hysteresis smoothly using exponential decay
5. THE System SHALL ensure stall transitions are smooth, not abrupt
6. THE System SHALL ensure transonic transitions are smooth
7. THE System SHALL ensure all derivatives are continuous
8. THE System SHALL avoid discontinuities that could cause integration instability

### Requirement 16: Edge Cases and Error Conditions

**User Story:** As a developer, I want robust error handling, so that I can handle edge cases gracefully.

#### Acceptance Criteria

1. WHEN velocity approaches zero, THE System SHALL throw SingularError before calculation
2. WHEN density approaches zero, THE System SHALL throw SingularError before calculation
3. WHEN Mach number equals exactly 1.0, THE System SHALL handle transonic interpolation smoothly
4. WHEN angle of attack equals exactly 90 degrees, THE System SHALL throw RangeError
5. WHEN Reynolds number is very small, THE System SHALL clamp to minimum value
6. WHEN aspect ratio is very small, THE System SHALL handle lift calculations without division by zero
7. WHEN component area is zero, THE System SHALL handle scaling without division by zero
8. WHEN all components have zero normal force, THE System SHALL compute center of pressure without division by zero

### Requirement 17: Force and Moment Conversion

**User Story:** As a developer, I want dimensional force conversion, so that I can get forces in Newtons.

#### Acceptance Criteria

1. WHEN converting coefficients to forces, THE System SHALL multiply by dynamic pressure and reference area
2. WHEN converting coefficients to moments, THE System SHALL multiply by dynamic pressure, reference area, and reference length
3. THE System SHALL calculate dynamic pressure as 0.5 * rho * V^2
4. THE System SHALL use appropriate reference lengths for each moment axis
5. THE System SHALL return forces as a 3D vector (Fx, Fy, Fz)
6. THE System SHALL return moments as a 3D vector (Mx, My, Mz)

### Requirement 18: Test Framework Integration

**User Story:** As a developer, I want Catch2 integration, so that I can run tests easily.

#### Acceptance Criteria

1. THE System SHALL use Catch2 framework for all unit tests
2. THE System SHALL use Catch2 framework for all property-based tests
3. THE System SHALL organize tests into logical test cases by component
4. THE System SHALL use descriptive test names that explain what is being tested
5. THE System SHALL use Catch2 sections to group related assertions
6. THE System SHALL use Catch2 generators for property-based testing
7. THE System SHALL run property tests with minimum 100 iterations
8. THE System SHALL tag property tests with feature name and property number

### Requirement 19: Reference Data Validation

**User Story:** As a developer, I want validation against reference data, so that I can verify calculation accuracy.

#### Acceptance Criteria

1. THE System SHALL validate body drag coefficients against published data for standard shapes
2. THE System SHALL validate wing lift slopes against thin airfoil theory
3. THE System SHALL validate transonic drag rise against experimental data
4. THE System SHALL validate supersonic lift slopes against linearized theory
5. THE System SHALL validate center of pressure locations against known configurations
6. THE System SHALL document all reference data sources with citations
7. THE System SHALL achieve agreement within 10% for well-established cases
8. THE System SHALL identify and document cases where agreement is outside tolerance

### Requirement 20: Regression Testing

**User Story:** As a developer, I want regression tests, so that I can prevent breaking changes.

#### Acceptance Criteria

1. THE System SHALL maintain a suite of regression test cases with known outputs
2. WHEN code changes are made, THE System SHALL verify outputs match previous results
3. THE System SHALL test complete calculation pipeline from JSON to output
4. THE System SHALL test multiple vehicle configurations (rocket, aircraft, missile)
5. THE System SHALL test across multiple flight regimes (subsonic, transonic, supersonic, hypersonic)
6. THE System SHALL test edge cases (zero alpha, high alpha, zero beta, high beta)
7. THE System SHALL store regression test data in version control
8. THE System SHALL document any intentional changes to regression test outputs
