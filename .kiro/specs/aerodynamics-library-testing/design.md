# Design Document: Aerodynamics Library Testing

## Overview

This design specifies a comprehensive testing strategy for the aero_simpi C++ aerodynamics library. The testing approach combines unit tests for specific behaviors, integration tests for complete workflows, and property-based tests for mathematical correctness. The design leverages Catch2's generator framework to implement property-based testing with minimum 100 iterations per property.

The testing strategy validates:
- JSON parsing and configuration validation
- Component factory pattern and polymorphism
- Aerodynamic calculations for body, wing, and fin components
- Mach regime transitions and interpolation
- Component interference effects
- Force and moment summation
- Mathematical properties (symmetry, continuity, physical bounds)
- Integration with reference data from NACA/NASA sources

## Architecture

### Test Organization

```
tests/
├── unit/
│   ├── test_parser.cpp          # JSON parsing tests
│   ├── test_config.cpp          # Configuration validation tests
│   ├── test_component.cpp       # Component factory and base class tests
│   ├── test_body.cpp            # Body component tests
│   ├── test_wing.cpp            # Wing component tests
│   ├── test_fin.cpp             # Fin component tests
│   └── test_model.cpp           # Model integration tests
├── property/
│   ├── test_symmetry.cpp        # Symmetry property tests
│   ├── test_continuity.cpp      # Continuity property tests
│   ├── test_bounds.cpp          # Physical bounds property tests
│   └── test_roundtrip.cpp       # Round-trip property tests
├── integration/
│   ├── test_pipeline.cpp        # End-to-end pipeline tests
│   └── test_regression.cpp      # Regression tests with known outputs
├── reference/
│   ├── test_validation.cpp      # Validation against reference data
│   └── reference_data/          # Reference data files
│       ├── naca0012_lift.csv
│       ├── body_drag.csv
│       └── transonic_drag.csv
└── generators/
    ├── state_generator.h        # AeroState generators
    ├── config_generator.h       # AeroConfig generators
    └── value_generator.h        # Primitive value generators
```

### Testing Framework

The design uses Catch2 v3.x with the following features:
- **TEST_CASE**: Defines individual test cases
- **SECTION**: Groups related assertions within a test case
- **GENERATE**: Creates data generators for property-based testing
- **REQUIRE**: Assertions that must pass
- **REQUIRE_THAT**: Matcher-based assertions for floating-point comparisons
- **Approx**: Floating-point comparison with tolerance

### Generator Architecture

Custom generators extend Catch2's generator interface to produce:
- Random aerodynamic states within valid ranges
- Random configurations with valid parameters
- Edge case values (boundaries, singularities)
- Structured test data (symmetric configurations, specific Mach regimes)

## Components and Interfaces

### Test Generators

#### StateGenerator
Generates valid AeroState objects with configurable ranges:

```cpp
class StateGenerator : public Catch::Generators::IGenerator<AeroState> {
public:
    struct Ranges {
        std::pair<double, double> V{10.0, 300.0};      // m/s
        std::pair<double, double> alpha{-30.0, 30.0};  // degrees
        std::pair<double, double> beta{-15.0, 15.0};   // degrees
        std::pair<double, double> M{0.1, 4.0};         // Mach number
        std::pair<double, double> rho{0.5, 1.5};       // kg/m³
        std::pair<double, double> Re{1e5, 1e7};        // Reynolds number
    };
    
    explicit StateGenerator(Ranges ranges, size_t count = 100);
    AeroState const& get() const override;
    bool next() override;
    
private:
    Ranges ranges_;
    size_t count_;
    size_t current_;
    AeroState current_state_;
    std::mt19937 rng_;
};

// Helper function for use in tests
Catch::Generators::GeneratorWrapper<AeroState> 
randomStates(StateGenerator::Ranges ranges = {}, size_t count = 100);
```

#### ConfigGenerator
Generates valid AeroConfig objects:

```cpp
class ConfigGenerator : public Catch::Generators::IGenerator<AeroConfig> {
public:
    enum class ConfigType {
        ROCKET,      // Body + fins
        AIRCRAFT,    // Body + wing + fins
        SIMPLE_BODY, // Body only
        CUSTOM       // Random configuration
    };
    
    explicit ConfigGenerator(ConfigType type, size_t count = 10);
    AeroConfig const& get() const override;
    bool next() override;
    
private:
    ConfigType type_;
    size_t count_;
    size_t current_;
    AeroConfig current_config_;
    std::mt19937 rng_;
};

Catch::Generators::GeneratorWrapper<AeroConfig> 
randomConfigs(ConfigGenerator::ConfigType type, size_t count = 10);
```

#### EdgeCaseGenerator
Generates edge case values for specific parameters:

```cpp
template<typename T>
class EdgeCaseGenerator : public Catch::Generators::IGenerator<T> {
public:
    explicit EdgeCaseGenerator(std::vector<T> values);
    T const& get() const override;
    bool next() override;
    
private:
    std::vector<T> values_;
    size_t current_;
};

// Predefined edge cases
auto machEdgeCases() -> Catch::Generators::GeneratorWrapper<double>;
// Returns: {0.1, 0.8, 0.85, 0.95, 1.0, 1.05, 1.2, 3.0, 4.0}

auto alphaEdgeCases() -> Catch::Generators::GeneratorWrapper<double>;
// Returns: {-90.0, -30.0, -15.0, 0.0, 15.0, 30.0, 90.0}
```

### Test Utilities

#### ApproxVector
Custom matcher for comparing 3D vectors with tolerance:

```cpp
class ApproxVector {
public:
    explicit ApproxVector(const Eigen::Vector3d& expected, double epsilon = 1e-6);
    bool match(const Eigen::Vector3d& actual) const;
    std::string describe() const;
    
private:
    Eigen::Vector3d expected_;
    double epsilon_;
};

// Usage in tests:
REQUIRE_THAT(actual_vector, ApproxVector(expected_vector, 1e-5));
```

#### ReferenceDataLoader
Loads and interpolates reference data from CSV files:

```cpp
class ReferenceDataLoader {
public:
    struct DataPoint {
        double x;
        double y;
    };
    
    explicit ReferenceDataLoader(const std::string& filepath);
    double interpolate(double x) const;
    std::vector<DataPoint> getData() const;
    
private:
    std::vector<DataPoint> data_;
    void loadCSV(const std::string& filepath);
};
```

#### PhysicalBoundsChecker
Validates physical realism of outputs:

```cpp
class PhysicalBoundsChecker {
public:
    static bool isValidDragCoefficient(double Cx);
    static bool isValidLiftToDragRatio(double CL, double CD);
    static bool isValidCenterOfPressure(double x_cp, double vehicle_length);
    static bool isValidStaticMargin(double static_margin);
    static bool isValidStallFactor(double factor);
    static bool isValidMachWeight(double weight);
};
```

## Data Models

### Test Configuration

Test configurations are defined in JSON files for regression testing:

```json
{
  "test_name": "rocket_subsonic_zero_alpha",
  "config_file": "config/aerodynamics_rocket_1m.json",
  "state": {
    "V": 100.0,
    "alpha": 0.0,
    "beta": 0.0,
    "M": 0.3,
    "rho": 1.225,
    "Re": 1e6,
    "x_com": 0.5
  },
  "expected_output": {
    "Cx": 0.15,
    "Cy": 0.0,
    "Cz": 0.0,
    "tolerance": 0.01
  }
}
```

### Reference Data Format

Reference data CSV files follow this format:

```csv
# NACA 0012 Lift Coefficient vs Angle of Attack
# Source: NASA Langley Research Center
# Reynolds Number: 3e6, Mach: 0.3
alpha,CL
-10.0,-1.05
-5.0,-0.52
0.0,0.0
5.0,0.52
10.0,1.05
15.0,1.35
```

## Correctness Properties

*A property is a characteristic or behavior that should hold true across all valid executions of a system—essentially, a formal statement about what the system should do. Properties serve as the bridge between human-readable specifications and machine-verifiable correctness guarantees.*


### Parsing and Configuration Properties

Property 1: JSON Round-Trip Preservation
*For any* valid AeroConfig object, serializing to JSON then parsing back should produce an equivalent configuration with all parameters preserved.
**Validates: Requirements 1.2, 1.10**

Property 2: Valid JSON Parsing Success
*For any* syntactically valid JSON file containing all required fields with correct types, the Parser should successfully parse it into an AeroConfig object without throwing exceptions.
**Validates: Requirements 1.1**

Property 3: Invalid JSON Error Handling
*For any* JSON file with missing required fields, invalid field types, or malformed syntax, the Parser should throw a ConfigError with a descriptive message.
**Validates: Requirements 1.3, 1.4, 1.8**

Property 4: Post-Parse Validation
*For any* JSON that successfully parses but contains invalid parameter values (e.g., negative lengths), the validation step should throw a ConfigError.
**Validates: Requirements 1.9**

### Configuration Validation Properties

Property 5: Invalid Global Parameters Rejection
*For any* configuration with S_ref, c_ref, or b_ref that is zero or negative, the validation should throw a ConfigError.
**Validates: Requirements 2.1, 2.2, 2.3**

Property 6: Invalid Component Parameters Rejection
*For any* component configuration with invalid geometric parameters (zero or negative length, diameter, or S_ref for the appropriate component type), the validation should throw a ConfigError.
**Validates: Requirements 2.5, 2.6, 2.7, 2.8**

Property 7: Valid Configuration Acceptance
*For any* randomly generated configuration with all parameters within valid ranges, the validation should succeed without throwing exceptions.
**Validates: Requirements 2.9**

### State Validation Properties

Property 8: Invalid State Parameters Rejection
*For any* state with V ≤ 0, rho ≤ 0, M < 0, M > 5.0, |alpha| > 90°, or |beta| > 90°, the validation should throw the appropriate error (SingularError or RangeError).
**Validates: Requirements 3.1, 3.2, 3.3, 3.4, 3.5**

Property 9: Valid State Acceptance
*For any* randomly generated state with all parameters within valid ranges, the validation should succeed without throwing exceptions.
**Validates: Requirements 3.6**

### Component Factory Properties

Property 10: Correct Component Type Construction
*For any* valid component configuration, the ComponentFactory should create an instance of the correct component type (BodyComponent for BODY, WingComponent for WING, FinComponent for FIN) that accepts the configuration without errors.
**Validates: Requirements 4.4, 4.5, 4.6**

### Body Component Properties

Property 11: Body Drag Positivity
*For any* body component and valid aerodynamic state, the calculated drag coefficient Cx should be strictly positive.
**Validates: Requirements 5.1, 13.1**

Property 12: Body Normal Force Monotonicity
*For any* body component and two states differing only in angle of attack where |alpha2| > |alpha1| (both in linear regime < 30°), the magnitude of normal force |Cz2| should be greater than |Cz1|.
**Validates: Requirements 5.2**

Property 13: Body Wave Drag Increase
*For any* body component and two states differing only in Mach number where M2 > M1 > 0.8, the drag coefficient Cx2 should be greater than Cx1.
**Validates: Requirements 5.3**

Property 14: Body Symmetry at Zero Angles
*For any* body component and state with alpha = 0 and beta = 0, the normal force Cz and side force Cy should both be approximately zero (within tolerance).
**Validates: Requirements 5.4, 5.6**

Property 15: Body Sign Conventions
*For any* body component and state with alpha > 0, the normal force Cz should be negative (Z-UP convention), and for beta ≠ 0, side force Cy should have appropriate sign.
**Validates: Requirements 5.5, 5.7**

Property 16: Body Stall Flag Setting
*For any* body component and state with |alpha| > 30°, the output should have is_body_stalled flag set to true.
**Validates: Requirements 5.9**

Property 17: Body Center of Pressure Variation
*For any* body component and two states with different Mach numbers or angles of attack, the center of pressure x_cp should vary (not be constant).
**Validates: Requirements 5.10**

### Wing Component Properties

Property 18: Wing Lift Linearity in Unstalled Regime
*For any* wing component and state in unstalled regime (|alpha| < stall_angle), the lift coefficient should be approximately linear with angle of attack (dCz/dalpha approximately constant).
**Validates: Requirements 6.1, 6.2**

Property 19: Wing Stall Behavior
*For any* wing component and state with |alpha| > stall_angle, the lift coefficient magnitude should be less than what linear extrapolation would predict, and is_wing_stalled flag should be true.
**Validates: Requirements 6.3, 6.4**

Property 20: Wing Mach Regime Corrections
*For any* wing component, lift slope at subsonic Mach (M < 0.8) should be higher than incompressible theory due to Prandtl-Glauert correction, and lift slope at supersonic Mach (M > 1.2) should follow supersonic theory.
**Validates: Requirements 6.5, 6.6**

Property 21: Wing Transonic Continuity
*For any* wing component and sequence of states with Mach numbers varying through transonic regime (0.7 to 1.3), all output coefficients should vary continuously without jumps.
**Validates: Requirements 6.7**

Property 22: Wing Hypersonic Effectiveness Reduction
*For any* wing component and two states with M2 = 4.0 and M1 = 2.0 (same alpha), the lift slope magnitude at M2 should be less than at M1.
**Validates: Requirements 6.8**

Property 23: Wing Vortex Lift Addition
*For any* wing component and state with |alpha| > 15°, the lift coefficient magnitude should be higher than linear theory would predict due to vortex lift.
**Validates: Requirements 6.9**

Property 24: Wing Induced Drag Relationship
*For any* wing component and state, the induced drag component should be approximately proportional to the square of the lift coefficient.
**Validates: Requirements 6.10**

### Fin Component Properties

Property 25: Fin Control Effectiveness
*For any* fin component and two states differing only in deflection angle (delta1 = 0, delta2 ≠ 0), the forces and moments should differ, with the difference proportional to delta2.
**Validates: Requirements 7.1, 7.2**

Property 26: Fin Force Projection by Mount Angle
*For any* fin component with mount_angle = 0°, |Cz| should be much larger than |Cy|, and for mount_angle = 90°, |Cy| should be much larger than |Cz|.
**Validates: Requirements 7.3, 7.4**

Property 27: Fin Downwash Effect
*For any* fin component and state with non-zero epsilon_prev (downwash), the effective angle of attack should be reduced compared to the geometric angle of attack.
**Validates: Requirements 7.5**

Property 28: Fin Transonic Effectiveness Reduction
*For any* fin component and two states with M1 = 0.5 and M2 = 1.0 (same alpha, delta), control effectiveness at M2 should be less than at M1.
**Validates: Requirements 7.6**

Property 29: Fin Moment Arm Scaling
*For any* fin component and state, moments should scale proportionally with the moment arm (x_pos - x_com).
**Validates: Requirements 7.8**

Property 30: Fin Area Scaling
*For any* fin component and state, forces should scale proportionally with the ratio S_ref / S_ref_global.
**Validates: Requirements 7.10**

### Mach Weight Properties

Property 31: Mach Weight Regime Assignment
*For any* Mach number M < 0.85, subsonic weight should be close to 1.0; for M > 0.95, supersonic weight should be close to 1.0; for M > 3.0, hypersonic weight should be close to 1.0.
**Validates: Requirements 8.1, 8.2, 8.3**

Property 32: Mach Weight Continuity
*For any* sequence of Mach numbers, all weight functions should vary continuously and smoothly (no discontinuities or sharp transitions).
**Validates: Requirements 8.4, 8.5, 8.6**

Property 33: Mach Weight Bounds
*For any* Mach number, all weight values should be in the range [0.0, 1.0].
**Validates: Requirements 13.6**

### Interference Properties

Property 34: Wing-Fuselage Interference Application
*For any* model configuration containing both a wing and a body, the wing lift coefficient should be increased by the interference factor compared to a wing-only configuration.
**Validates: Requirements 9.1, 9.2**

Property 35: Wake Shadow Effect
*For any* model configuration containing both a wing and a fin, the fin forces should be reduced based on the wing's lift coefficient, with higher wing lift causing greater reduction.
**Validates: Requirements 9.3, 9.4**

Property 36: Wake Shadow Bounds
*For any* model configuration with wake shadow applied, the dynamic pressure ratio should never fall below 0.5.
**Validates: Requirements 9.6**

Property 37: Conditional Interference Application
*For any* model configuration without a wing, fins should not have wake shadow applied; for any configuration without a body, wings should not have interference factor applied.
**Validates: Requirements 9.7, 9.8**

### Model Integration Properties

Property 38: Force and Moment Summation
*For any* model and state, the total force coefficients (Cx, Cy, Cz) and moment coefficients (mx, my, mz) should equal the sum of contributions from all components (after interference is applied).
**Validates: Requirements 10.1, 10.2, 10.6**

Property 39: Center of Pressure Calculation
*For any* model and state with non-zero total normal force, the center of pressure should be the weighted average of component centers of pressure, weighted by their normal force magnitudes.
**Validates: Requirements 10.3**

Property 40: Center of Pressure Edge Case
*For any* model and state with total normal force magnitude less than 1e-6, the center of pressure should be calculated as the geometric average of component positions.
**Validates: Requirements 10.4**

Property 41: Static Margin Formula
*For any* model output, the static_margin field should equal (x_cp - x_com) / c_ref within numerical tolerance.
**Validates: Requirements 10.5**

Property 42: Transonic Flag Setting
*For any* model and state with 0.8 ≤ M ≤ 1.2, the output should have is_transonic flag set to true.
**Validates: Requirements 10.7**

Property 43: Stall Flag Propagation
*For any* model and state, if any component output has is_body_stalled or is_wing_stalled set to true, the total output should also have the corresponding flag set to true.
**Validates: Requirements 10.8**

### Derivative Properties

Property 44: Static Derivatives Calculation
*For any* model and state, all static derivatives (dCx_dalpha, dCz_dalpha, dmy_dalpha, dCy_dbeta, dmz_dbeta, dCz_ddelta, dmy_ddelta) should be calculated, finite, and equal to the sum of component derivatives.
**Validates: Requirements 11.1, 11.2, 11.3, 11.4, 11.5, 11.6, 11.7, 11.9, 11.10**

Property 45: Dynamic Derivatives Calculation
*For any* model and state with V > 0, all dynamic derivatives (dCz_dq, dmy_dq, dCy_dr, dmz_dr, dmx_dp) should be calculated, finite, and scale appropriately with moment arm and velocity.
**Validates: Requirements 12.1, 12.2, 12.3, 12.4, 12.5, 12.7, 12.8**

### Physical Bounds Properties

Property 46: Drag Coefficient Positivity
*For any* valid calculation (any component or model, any valid state), the drag coefficient Cx should always be strictly positive.
**Validates: Requirements 13.1**

Property 47: Lift-to-Drag Ratio Bounds
*For any* valid calculation with non-zero drag, the absolute value of lift-to-drag ratio |CL/CD| should be less than 20 (physically realistic bound).
**Validates: Requirements 13.2**

Property 48: Center of Pressure Bounds
*For any* model with known vehicle length, the center of pressure x_cp should be within the range [0, vehicle_length].
**Validates: Requirements 13.3**

Property 49: Static Margin Bounds
*For any* model output, the static margin should be within the range [-2.0, 5.0] (physically reasonable bounds).
**Validates: Requirements 13.4**

Property 50: Stall Factor Bounds
*For any* wing or fin component calculation, the stall factor should be within the range [0.4, 1.0].
**Validates: Requirements 13.5**

Property 51: Interference Factor Positivity
*For any* model with interference effects, all interference factors should be strictly positive.
**Validates: Requirements 13.7**

### Symmetry Properties

Property 52: Angle of Attack Symmetry
*For any* symmetric component or model and two states differing only in the sign of alpha (alpha2 = -alpha1), the normal force and pitching moment should be negated: Cz2 = -Cz1 and my2 = -my1.
**Validates: Requirements 14.1, 14.2**

Property 53: Sideslip Angle Symmetry
*For any* symmetric component or model and two states differing only in the sign of beta (beta2 = -beta1), the side force and yawing moment should be negated: Cy2 = -Cy1 and mz2 = -mz1.
**Validates: Requirements 14.3, 14.4**

Property 54: Zero Angle Symmetry
*For any* symmetric component or model and state with alpha = 0 and beta = 0, both side force Cy and normal force Cz should be approximately zero (within tolerance).
**Validates: Requirements 14.5, 14.6**

Property 55: Control Deflection Symmetry
*For any* fin component and two states differing only in the sign of deflection (delta2 = -delta1), the control forces should be approximately negated.
**Validates: Requirements 14.7**

### Continuity Properties

Property 56: Mach Number Continuity
*For any* component or model and sequence of states with Mach numbers varying continuously (including through transonic regime), all output coefficients should vary continuously without discontinuities.
**Validates: Requirements 15.1, 15.3, 15.6**

Property 57: Angle of Attack Continuity
*For any* component or model and sequence of states with angle of attack varying continuously (including through stall), all output coefficients should vary continuously without abrupt jumps.
**Validates: Requirements 15.2, 15.5**

Property 58: Derivative Continuity
*For any* component or model and sequence of states varying continuously in any parameter, all derivative values should also vary continuously.
**Validates: Requirements 15.7**

### Force Conversion Properties

Property 59: Dimensional Force Conversion
*For any* output and given reference parameters (S_ref, c_ref, b_ref, V, rho), the dimensional forces calculated by getForcesAndMoments should equal coefficients multiplied by dynamic pressure and appropriate reference dimensions.
**Validates: Requirements 17.1, 17.2, 17.3, 17.4, 17.5, 17.6**


## Error Handling

### Exception Hierarchy

The library defines three exception types, all derived from AeroException:
- **ConfigError**: Configuration and parsing errors
- **SingularError**: Singular conditions (V=0, rho=0)
- **RangeError**: Out-of-range parameters

### Error Testing Strategy

1. **Validation Errors**: Test that invalid configurations and states throw appropriate errors
2. **Edge Cases**: Test boundary conditions (M=1.0, alpha=90°, etc.)
3. **Singularities**: Test that singular conditions are caught before calculation
4. **Error Messages**: Verify error messages are descriptive and include problematic values

### Error Test Examples

```cpp
TEST_CASE("Configuration validation catches invalid parameters", "[config][validation]") {
    SECTION("Zero reference area throws ConfigError") {
        AeroConfig config;
        config.global.S_ref = 0.0;  // Invalid
        config.global.c_ref = 1.0;
        config.global.b_ref = 1.0;
        
        REQUIRE_THROWS_AS(config.validate(), aero::ConfigError);
    }
    
    SECTION("Negative body length throws ConfigError") {
        ComponentConfig body;
        body.type = ComponentType::BODY;
        body.length = -1.0;  // Invalid
        body.diameter = 0.1;
        
        REQUIRE_THROWS_AS(body.validate(), aero::ConfigError);
    }
}

TEST_CASE("State validation catches singular conditions", "[state][validation]") {
    SECTION("Zero velocity throws SingularError") {
        AeroState state;
        state.V = 0.0;  // Singular
        state.rho = 1.225;
        state.M = 0.0;
        
        REQUIRE_THROWS_AS(state.validate(), aero::SingularError);
    }
    
    SECTION("Out of range Mach throws RangeError") {
        AeroState state;
        state.V = 100.0;
        state.rho = 1.225;
        state.M = 6.0;  // Out of range
        
        REQUIRE_THROWS_AS(state.validate(), aero::RangeError);
    }
}
```

## Testing Strategy

### Dual Testing Approach

The testing strategy employs both unit tests and property-based tests as complementary approaches:

**Unit Tests**:
- Verify specific examples and known cases
- Test edge cases and boundary conditions
- Test error handling with specific invalid inputs
- Validate against reference data from NACA/NASA sources
- Test integration points between components
- Regression tests with stored expected outputs

**Property-Based Tests**:
- Verify universal properties across randomly generated inputs
- Test mathematical properties (symmetry, continuity, monotonicity)
- Test physical constraints (positive drag, bounded ratios)
- Test invariants (round-trip preservation, summation correctness)
- Run minimum 100 iterations per property test
- Use Catch2 GENERATE for input generation

### Property-Based Testing Configuration

Each property test must:
1. Use Catch2 GENERATE to create random inputs (minimum 100 iterations)
2. Include a comment tag referencing the design property
3. Use descriptive test names explaining the property being tested
4. Use appropriate floating-point tolerances for comparisons

**Tag Format**:
```cpp
// Feature: aerodynamics-library-testing, Property 52: Angle of Attack Symmetry
TEST_CASE("Angle of attack symmetry property", "[property][symmetry][alpha]") {
    auto state = GENERATE(take(100, randomStates()));
    // ... test implementation
}
```

### Test Organization by Type

**Unit Tests** (tests/unit/):
- test_parser.cpp: JSON parsing, file I/O, error cases
- test_config.cpp: Configuration validation, factory pattern
- test_component.cpp: Component base class, polymorphism
- test_body.cpp: Body-specific calculations, edge cases
- test_wing.cpp: Wing-specific calculations, stall behavior
- test_fin.cpp: Fin-specific calculations, control effectiveness
- test_model.cpp: Model integration, summation, interference

**Property Tests** (tests/property/):
- test_symmetry.cpp: Properties 52-55 (angle symmetry, control symmetry)
- test_continuity.cpp: Properties 56-58 (Mach continuity, alpha continuity)
- test_bounds.cpp: Properties 46-51 (physical bounds, positivity)
- test_roundtrip.cpp: Property 1 (JSON round-trip preservation)

**Integration Tests** (tests/integration/):
- test_pipeline.cpp: End-to-end tests from JSON to output
- test_regression.cpp: Regression tests with stored expected values

**Reference Validation Tests** (tests/reference/):
- test_validation.cpp: Validation against NACA/NASA data
- Reference data files in tests/reference/reference_data/

### Reference Data Sources

The following reference data sources will be used for validation:

1. **NACA 0012 Airfoil Data**
   - Source: NASA Langley Research Center Turbulence Modeling Resource
   - Data: Lift coefficient vs angle of attack at Re = 3×10⁶, M = 0.3
   - URL: https://turbmodels.larc.nasa.gov/naca0012_val.html
   - Expected agreement: Within 10% for linear regime

2. **Body Drag Coefficients**
   - Source: NACA Technical Reports on bodies of revolution
   - Data: Drag coefficients for ogive, cone, and sphere nose shapes
   - Expected agreement: Within 15% (due to simplified model)

3. **Transonic Drag Rise**
   - Source: NACA Research Memoranda on transonic aerodynamics
   - Data: Drag coefficient vs Mach number through transonic regime
   - Expected agreement: Qualitative trend matching (drag rise near M=1)

4. **Supersonic Lift Slopes**
   - Source: Linearized supersonic theory (analytical)
   - Data: CL_alpha = 4 / sqrt(M² - 1) for thin airfoils
   - Expected agreement: Within 20% (due to finite thickness effects)

### Test Execution Strategy

**Development Testing**:
- Run unit tests frequently during development
- Run property tests before commits
- Use Catch2 tags to run specific test subsets

**Continuous Integration**:
- Run all unit tests on every commit
- Run all property tests on every commit
- Run integration tests on every commit
- Run reference validation tests weekly (slower, requires data files)

**Performance Considerations**:
- Property tests with 100 iterations may take longer
- Use Catch2's parallel execution when available
- Cache reference data loading for multiple tests

### Example Property Test Implementation

```cpp
// Feature: aerodynamics-library-testing, Property 52: Angle of Attack Symmetry
TEST_CASE("Angle of attack symmetry property", "[property][symmetry][alpha]") {
    // Generate random valid configurations
    auto config = GENERATE(take(10, randomConfigs(ConfigGenerator::ROCKET)));
    auto model = AerodynamicsModel::create(config);
    
    // Generate random valid states
    auto state = GENERATE(take(100, randomStates()));
    
    // Calculate at +alpha
    auto output_pos = model->calculate(state);
    
    // Calculate at -alpha
    AeroState state_neg = state;
    state_neg.alpha = -state.alpha;
    auto output_neg = model->calculate(state_neg);
    
    // Verify symmetry: Cz(-alpha) = -Cz(+alpha)
    REQUIRE_THAT(output_neg.Cz, Catch::Matchers::WithinAbs(-output_pos.Cz, 1e-6));
    
    // Verify symmetry: my(-alpha) = -my(+alpha)
    REQUIRE_THAT(output_neg.my, Catch::Matchers::WithinAbs(-output_pos.my, 1e-6));
}
```

### Example Unit Test with Reference Data

```cpp
TEST_CASE("Wing lift slope matches NACA 0012 data", "[reference][wing][lift]") {
    // Load reference data
    ReferenceDataLoader ref_data("tests/reference/reference_data/naca0012_lift.csv");
    
    // Create wing configuration matching reference conditions
    ComponentConfig wing_config;
    wing_config.type = ComponentType::WING;
    wing_config.S_ref = 1.0;
    wing_config.AR = 6.0;
    wing_config.c_ref = 1.0;
    wing_config.b_ref = 6.0;
    
    auto wing = std::make_shared<WingComponent>(wing_config);
    
    GlobalConfig global;
    global.S_ref = 1.0;
    global.c_ref = 1.0;
    global.b_ref = 6.0;
    
    // Test at multiple angles of attack
    for (double alpha = -10.0; alpha <= 10.0; alpha += 2.0) {
        AeroState state;
        state.V = 100.0;
        state.M = 0.3;
        state.Re = 3e6;
        state.rho = 1.225;
        state.alpha = alpha;
        state.x_com = 0.25;
        
        auto output = wing->calculate(state, global);
        double CL_calculated = -output.Cz;  // Z-UP convention
        double CL_reference = ref_data.interpolate(alpha);
        
        // Verify within 10% agreement
        double relative_error = std::abs(CL_calculated - CL_reference) / std::abs(CL_reference);
        REQUIRE(relative_error < 0.10);
    }
}
```

### Test Coverage Goals

- **Line Coverage**: Minimum 90% of library code
- **Branch Coverage**: Minimum 85% of conditional branches
- **Property Coverage**: All testable acceptance criteria have corresponding property tests
- **Reference Validation**: At least 4 reference data sets validated
- **Regression Coverage**: At least 20 regression test cases covering all flight regimes

### Testing Tools and Infrastructure

**Build System**:
- CMake integration for test compilation
- Separate test executable from library
- Link against Catch2 (already in libs/Catch2)

**Test Execution**:
- Catch2 command-line interface for test selection
- XML output for CI integration
- Console reporter for development

**Coverage Analysis**:
- gcov/lcov for coverage measurement
- HTML coverage reports
- Coverage tracking in CI

**Continuous Integration**:
- Automated test execution on commits
- Coverage reporting
- Performance regression detection

## Implementation Notes

### Floating-Point Comparisons

All floating-point comparisons in tests must use appropriate tolerances:
- **Absolute tolerance**: For values near zero (e.g., Cz at alpha=0)
- **Relative tolerance**: For larger values (e.g., drag coefficients)
- **Default tolerance**: 1e-6 for most comparisons
- **Relaxed tolerance**: 1e-3 for complex calculations with accumulated error

### Random Number Generation

Property-based tests use deterministic random number generation:
- Fixed seed for reproducibility
- Seed can be overridden via command-line for exploration
- Document seed in test output for debugging failures

### Test Data Management

Reference data files:
- Stored in version control (tests/reference/reference_data/)
- CSV format for easy inspection and modification
- Include metadata comments (source, conditions, units)
- Automated validation of data file format

### Performance Considerations

Property tests with 100 iterations:
- Expected runtime: 1-5 seconds per property test
- Total property test suite: ~5 minutes
- Unit test suite: ~30 seconds
- Integration test suite: ~1 minute
- Reference validation suite: ~2 minutes

Total test suite runtime: ~10 minutes (acceptable for CI)

## Appendix: Catch2 Generator Examples

### Basic Generator Usage

```cpp
// Generate random doubles in range
auto value = GENERATE(take(100, random(0.0, 10.0)));

// Generate from list of values
auto mach = GENERATE(values({0.3, 0.8, 1.0, 1.5, 3.0}));

// Generate combinations
auto alpha = GENERATE(range(-30.0, 30.0, 5.0));
auto beta = GENERATE(range(-15.0, 15.0, 5.0));
```

### Custom Generator Implementation

```cpp
class StateGenerator : public Catch::Generators::IGenerator<AeroState> {
public:
    StateGenerator(Ranges ranges, size_t count) 
        : ranges_(ranges), count_(count), current_(0) {
        // Initialize random number generator with fixed seed
        rng_.seed(42);
        // Generate first state
        generateNext();
    }
    
    AeroState const& get() const override {
        return current_state_;
    }
    
    bool next() override {
        if (++current_ >= count_) {
            return false;
        }
        generateNext();
        return true;
    }
    
private:
    void generateNext() {
        std::uniform_real_distribution<double> V_dist(ranges_.V.first, ranges_.V.second);
        std::uniform_real_distribution<double> alpha_dist(ranges_.alpha.first, ranges_.alpha.second);
        std::uniform_real_distribution<double> beta_dist(ranges_.beta.first, ranges_.beta.second);
        std::uniform_real_distribution<double> M_dist(ranges_.M.first, ranges_.M.second);
        std::uniform_real_distribution<double> rho_dist(ranges_.rho.first, ranges_.rho.second);
        std::uniform_real_distribution<double> Re_dist(ranges_.Re.first, ranges_.Re.second);
        
        current_state_.V = V_dist(rng_);
        current_state_.alpha = alpha_dist(rng_);
        current_state_.beta = beta_dist(rng_);
        current_state_.M = M_dist(rng_);
        current_state_.rho = rho_dist(rng_);
        current_state_.Re = Re_dist(rng_);
        current_state_.x_com = 0.5;  // Default center of mass
    }
    
    Ranges ranges_;
    size_t count_;
    size_t current_;
    AeroState current_state_;
    std::mt19937 rng_;
};

// Helper function for use in tests
inline Catch::Generators::GeneratorWrapper<AeroState> 
randomStates(StateGenerator::Ranges ranges = {}, size_t count = 100) {
    return Catch::Generators::GeneratorWrapper<AeroState>(
        std::make_unique<StateGenerator>(ranges, count)
    );
}
```

## Summary

This design provides a comprehensive testing strategy for the aero_simpi aerodynamics library, combining:

1. **Property-based testing** for mathematical correctness and universal properties
2. **Unit testing** for specific cases, edge conditions, and error handling
3. **Integration testing** for end-to-end workflows
4. **Reference validation** against NACA/NASA data for physical accuracy
5. **Regression testing** to prevent breaking changes

The testing approach ensures:
- Mathematical correctness through property verification
- Physical realism through bounds checking and reference validation
- Robustness through error handling and edge case testing
- Maintainability through clear organization and documentation
- Confidence through comprehensive coverage (>90% line coverage goal)

All tests use the Catch2 framework with custom generators for property-based testing, achieving minimum 100 iterations per property test to thoroughly explore the input space.
