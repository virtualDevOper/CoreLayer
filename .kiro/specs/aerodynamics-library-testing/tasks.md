# Implementation Plan: Aerodynamics Library Testing

## Overview

This implementation plan creates a comprehensive test suite for the aero_simpi C++ aerodynamics library using Catch2. The approach combines unit tests for specific behaviors, property-based tests for mathematical correctness, integration tests for workflows, and reference validation tests for physical accuracy. All tests will be organized in a clear directory structure and use Catch2's generator framework for property-based testing with minimum 100 iterations per property.

## Tasks

- [ ] 1. Set up test infrastructure and directory structure
  - Create tests/ directory with subdirectories (unit/, property/, integration/, reference/)
  - Create CMakeLists.txt for test compilation
  - Link against Catch2 framework (already in libs/Catch2)
  - Create main test runner file with Catch2 main
  - _Requirements: 18.1, 18.2, 18.3_

- [ ] 2. Implement custom test generators
  - [ ] 2.1 Create generators/state_generator.h with StateGenerator class
    - Implement IGenerator interface for AeroState
    - Support configurable ranges for V, alpha, beta, M, rho, Re
    - Use deterministic random number generation (fixed seed)
    - Implement randomStates() helper function
    - _Requirements: 18.6_
  
  - [ ] 2.2 Create generators/config_generator.h with ConfigGenerator class
    - Implement IGenerator interface for AeroConfig
    - Support ConfigType enum (ROCKET, AIRCRAFT, SIMPLE_BODY, CUSTOM)
    - Generate valid configurations for each type
    - Implement randomConfigs() helper function
    - _Requirements: 18.6_
  
  - [ ] 2.3 Create generators/value_generator.h with edge case generators
    - Implement machEdgeCases() returning {0.1, 0.8, 0.85, 0.95, 1.0, 1.05, 1.2, 3.0, 4.0}
    - Implement alphaEdgeCases() returning {-90.0, -30.0, -15.0, 0.0, 15.0, 30.0, 90.0}
    - Implement betaEdgeCases() for sideslip angles
    - _Requirements: 18.6_

- [ ] 3. Implement test utilities
  - [ ] 3.1 Create test_utils.h with ApproxVector matcher
    - Implement custom Catch2 matcher for Eigen::Vector3d comparison
    - Support configurable epsilon tolerance
    - Provide descriptive error messages
    - _Requirements: 18.5_
  
  - [ ] 3.2 Create test_utils.h with ReferenceDataLoader class
    - Implement CSV file loading
    - Implement linear interpolation for reference data
    - Handle comments and metadata in CSV files
    - _Requirements: 19.6_
  
  - [ ] 3.3 Create test_utils.h with PhysicalBoundsChecker class
    - Implement static methods for validating physical realism
    - Check drag coefficient, L/D ratio, center of pressure, static margin
    - Check stall factors, Mach weights, interference factors
    - _Requirements: 13.1, 13.2, 13.3, 13.4, 13.5, 13.6, 13.7_

- [ ] 4. Implement JSON parsing and configuration tests
  - [ ] 4.1 Create unit/test_parser.cpp with parsing tests
    - Test valid JSON parsing success
    - Test missing required fields throw ConfigError
    - Test invalid field types throw ConfigError
    - Test malformed JSON throws ConfigError
    - Test file not found throws ConfigError
    - Test invalid component types throw ConfigError (examples)
    - Test invalid nose types throw ConfigError (examples)
    - _Requirements: 1.1, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8_
  
  - [ ]* 4.2 Write property test for JSON round-trip preservation
    - **Property 1: JSON Round-Trip Preservation**
    - **Validates: Requirements 1.2, 1.10**
  
  - [ ]* 4.3 Write property test for valid JSON parsing
    - **Property 2: Valid JSON Parsing Success**
    - **Validates: Requirements 1.1**
  
  - [ ]* 4.4 Write property test for invalid JSON error handling
    - **Property 3: Invalid JSON Error Handling**
    - **Validates: Requirements 1.3, 1.4, 1.8**
  
  - [ ]* 4.5 Write property test for post-parse validation
    - **Property 4: Post-Parse Validation**
    - **Validates: Requirements 1.9**

- [ ] 5. Implement configuration validation tests
  - [ ] 5.1 Create unit/test_config.cpp with validation tests
    - Test zero/negative S_ref throws ConfigError
    - Test zero/negative c_ref throws ConfigError
    - Test zero/negative b_ref throws ConfigError
    - Test empty components list throws ConfigError (example)
    - Test zero/negative body length throws ConfigError
    - Test zero/negative body diameter throws ConfigError
    - Test zero/negative wing S_ref throws ConfigError
    - Test zero/negative fin S_ref throws ConfigError
    - _Requirements: 2.1, 2.2, 2.3, 2.4, 2.5, 2.6, 2.7, 2.8_
  
  - [ ]* 5.2 Write property test for invalid global parameters rejection
    - **Property 5: Invalid Global Parameters Rejection**
    - **Validates: Requirements 2.1, 2.2, 2.3**
  
  - [ ]* 5.3 Write property test for invalid component parameters rejection
    - **Property 6: Invalid Component Parameters Rejection**
    - **Validates: Requirements 2.5, 2.6, 2.7, 2.8**
  
  - [ ]* 5.4 Write property test for valid configuration acceptance
    - **Property 7: Valid Configuration Acceptance**
    - **Validates: Requirements 2.9**

- [ ] 6. Implement state validation tests
  - [ ] 6.1 Create unit/test_state.cpp with state validation tests
    - Test zero/negative velocity throws SingularError
    - Test zero/negative density throws SingularError
    - Test out-of-range Mach throws RangeError
    - Test out-of-range alpha throws RangeError
    - Test out-of-range beta throws RangeError
    - _Requirements: 3.1, 3.2, 3.3, 3.4, 3.5_
  
  - [ ]* 6.2 Write property test for invalid state parameters rejection
    - **Property 8: Invalid State Parameters Rejection**
    - **Validates: Requirements 3.1, 3.2, 3.3, 3.4, 3.5**
  
  - [ ]* 6.3 Write property test for valid state acceptance
    - **Property 9: Valid State Acceptance**
    - **Validates: Requirements 3.6**

- [ ] 7. Checkpoint - Ensure validation tests pass
  - Ensure all tests pass, ask the user if questions arise.

- [ ] 8. Implement component factory tests
  - [ ] 8.1 Create unit/test_component.cpp with factory tests
    - Test BODY config creates BodyComponent (example)
    - Test WING config creates WingComponent (example)
    - Test FIN config creates FinComponent (example)
    - Test BodyComponent rejects non-BODY config (example)
    - Test WingComponent rejects non-WING config (example)
    - Test FinComponent rejects non-FIN config (example)
    - _Requirements: 4.1, 4.2, 4.3, 4.7, 4.8, 4.9_
  
  - [ ]* 8.2 Write property test for correct component type construction
    - **Property 10: Correct Component Type Construction**
    - **Validates: Requirements 4.4, 4.5, 4.6**

- [ ] 9. Implement body component tests
  - [ ] 9.1 Create unit/test_body.cpp with body-specific tests
    - Test drag at zero alpha is positive
    - Test normal force increases with alpha
    - Test wave drag increases with Mach
    - Test zero normal force at alpha=0
    - Test negative Cz for positive alpha (Z-UP convention)
    - Test zero side force at beta=0
    - Test side force proportional to beta
    - Test hypersonic corrections at M>3
    - Test stall flag at |alpha|>30
    - Test center of pressure varies with M and alpha
    - _Requirements: 5.1, 5.2, 5.3, 5.4, 5.5, 5.6, 5.7, 5.8, 5.9, 5.10_
  
  - [ ]* 9.2 Write property test for body drag positivity
    - **Property 11: Body Drag Positivity**
    - **Validates: Requirements 5.1, 13.1**
  
  - [ ]* 9.3 Write property test for body normal force monotonicity
    - **Property 12: Body Normal Force Monotonicity**
    - **Validates: Requirements 5.2**
  
  - [ ]* 9.4 Write property test for body wave drag increase
    - **Property 13: Body Wave Drag Increase**
    - **Validates: Requirements 5.3**
  
  - [ ]* 9.5 Write property test for body symmetry at zero angles
    - **Property 14: Body Symmetry at Zero Angles**
    - **Validates: Requirements 5.4, 5.6**
  
  - [ ]* 9.6 Write property test for body sign conventions
    - **Property 15: Body Sign Conventions**
    - **Validates: Requirements 5.5, 5.7**
  
  - [ ]* 9.7 Write property test for body stall flag setting
    - **Property 16: Body Stall Flag Setting**
    - **Validates: Requirements 5.9**
  
  - [ ]* 9.8 Write property test for body center of pressure variation
    - **Property 17: Body Center of Pressure Variation**
    - **Validates: Requirements 5.10**

- [ ] 10. Implement wing component tests
  - [ ] 10.1 Create unit/test_wing.cpp with wing-specific tests
    - Test near-zero lift at alpha=0
    - Test linear lift in unstalled regime
    - Test reduced lift in stalled regime
    - Test stall flag setting
    - Test Prandtl-Glauert correction at subsonic Mach
    - Test supersonic lift theory at supersonic Mach
    - Test smooth transonic transition
    - Test hypersonic effectiveness reduction
    - Test vortex lift at high alpha
    - Test induced drag proportional to lift squared
    - _Requirements: 6.1, 6.2, 6.3, 6.4, 6.5, 6.6, 6.7, 6.8, 6.9, 6.10_
  
  - [ ]* 10.2 Write property test for wing lift linearity in unstalled regime
    - **Property 18: Wing Lift Linearity in Unstalled Regime**
    - **Validates: Requirements 6.1, 6.2**
  
  - [ ]* 10.3 Write property test for wing stall behavior
    - **Property 19: Wing Stall Behavior**
    - **Validates: Requirements 6.3, 6.4**
  
  - [ ]* 10.4 Write property test for wing Mach regime corrections
    - **Property 20: Wing Mach Regime Corrections**
    - **Validates: Requirements 6.5, 6.6**
  
  - [ ]* 10.5 Write property test for wing transonic continuity
    - **Property 21: Wing Transonic Continuity**
    - **Validates: Requirements 6.7**
  
  - [ ]* 10.6 Write property test for wing hypersonic effectiveness reduction
    - **Property 22: Wing Hypersonic Effectiveness Reduction**
    - **Validates: Requirements 6.8**
  
  - [ ]* 10.7 Write property test for wing vortex lift addition
    - **Property 23: Wing Vortex Lift Addition**
    - **Validates: Requirements 6.9**
  
  - [ ]* 10.8 Write property test for wing induced drag relationship
    - **Property 24: Wing Induced Drag Relationship**
    - **Validates: Requirements 6.10**

- [ ] 11. Checkpoint - Ensure component tests pass
  - Ensure all tests pass, ask the user if questions arise.

- [ ] 12. Implement fin component tests
  - [ ] 12.1 Create unit/test_fin.cpp with fin-specific tests
    - Test forces at zero deflection
    - Test control forces at non-zero deflection
    - Test force projection at mount_angle=0
    - Test force projection at mount_angle=90
    - Test downwash effect
    - Test transonic effectiveness reduction
    - Test stall effects
    - Test moment arm scaling
    - Test hysteresis application
    - Test area scaling
    - _Requirements: 7.1, 7.2, 7.3, 7.4, 7.5, 7.6, 7.7, 7.8, 7.9, 7.10_
  
  - [ ]* 12.2 Write property test for fin control effectiveness
    - **Property 25: Fin Control Effectiveness**
    - **Validates: Requirements 7.1, 7.2**
  
  - [ ]* 12.3 Write property test for fin force projection by mount angle
    - **Property 26: Fin Force Projection by Mount Angle**
    - **Validates: Requirements 7.3, 7.4**
  
  - [ ]* 12.4 Write property test for fin downwash effect
    - **Property 27: Fin Downwash Effect**
    - **Validates: Requirements 7.5**
  
  - [ ]* 12.5 Write property test for fin transonic effectiveness reduction
    - **Property 28: Fin Transonic Effectiveness Reduction**
    - **Validates: Requirements 7.6**
  
  - [ ]* 12.6 Write property test for fin moment arm scaling
    - **Property 29: Fin Moment Arm Scaling**
    - **Validates: Requirements 7.8**
  
  - [ ]* 12.7 Write property test for fin area scaling
    - **Property 30: Fin Area Scaling**
    - **Validates: Requirements 7.10**

- [ ] 13. Implement Mach weight tests
  - [ ] 13.1 Create unit/test_mach_weights.cpp with Mach weight tests
    - Test subsonic weight at M<0.85
    - Test supersonic weight at M>0.95
    - Test hypersonic weight at M>3.0
    - Test smooth transitions in transition zones
    - _Requirements: 8.1, 8.2, 8.3, 8.4_
  
  - [ ]* 13.2 Write property test for Mach weight regime assignment
    - **Property 31: Mach Weight Regime Assignment**
    - **Validates: Requirements 8.1, 8.2, 8.3**
  
  - [ ]* 13.3 Write property test for Mach weight continuity
    - **Property 32: Mach Weight Continuity**
    - **Validates: Requirements 8.4, 8.5, 8.6**
  
  - [ ]* 13.4 Write property test for Mach weight bounds
    - **Property 33: Mach Weight Bounds**
    - **Validates: Requirements 13.6**

- [ ] 14. Implement interference and model integration tests
  - [ ] 14.1 Create unit/test_model.cpp with model integration tests
    - Test wing-fuselage interference application
    - Test wake shadow effect
    - Test wake shadow bounds
    - Test conditional interference application
    - Test force and moment summation
    - Test center of pressure calculation
    - Test center of pressure edge case
    - Test static margin formula
    - Test transonic flag setting
    - Test stall flag propagation
    - _Requirements: 9.1, 9.2, 9.3, 9.4, 9.6, 9.7, 9.8, 10.1, 10.2, 10.3, 10.4, 10.5, 10.7, 10.8_
  
  - [ ]* 14.2 Write property test for wing-fuselage interference application
    - **Property 34: Wing-Fuselage Interference Application**
    - **Validates: Requirements 9.1, 9.2**
  
  - [ ]* 14.3 Write property test for wake shadow effect
    - **Property 35: Wake Shadow Effect**
    - **Validates: Requirements 9.3, 9.4**
  
  - [ ]* 14.4 Write property test for wake shadow bounds
    - **Property 36: Wake Shadow Bounds**
    - **Validates: Requirements 9.6**
  
  - [ ]* 14.5 Write property test for conditional interference application
    - **Property 37: Conditional Interference Application**
    - **Validates: Requirements 9.7, 9.8**
  
  - [ ]* 14.6 Write property test for force and moment summation
    - **Property 38: Force and Moment Summation**
    - **Validates: Requirements 10.1, 10.2, 10.6**
  
  - [ ]* 14.7 Write property test for center of pressure calculation
    - **Property 39: Center of Pressure Calculation**
    - **Validates: Requirements 10.3**
  
  - [ ]* 14.8 Write property test for center of pressure edge case
    - **Property 40: Center of Pressure Edge Case**
    - **Validates: Requirements 10.4**
  
  - [ ]* 14.9 Write property test for static margin formula
    - **Property 41: Static Margin Formula**
    - **Validates: Requirements 10.5**
  
  - [ ]* 14.10 Write property test for transonic flag setting
    - **Property 42: Transonic Flag Setting**
    - **Validates: Requirements 10.7**
  
  - [ ]* 14.11 Write property test for stall flag propagation
    - **Property 43: Stall Flag Propagation**
    - **Validates: Requirements 10.8**

- [ ] 15. Checkpoint - Ensure model integration tests pass
  - Ensure all tests pass, ask the user if questions arise.

- [ ] 16. Implement derivative tests
  - [ ] 16.1 Create unit/test_derivatives.cpp with derivative tests
    - Test all static derivatives are calculated and finite
    - Test static derivatives sum correctly across components
    - Test all dynamic derivatives are calculated and finite
    - Test dynamic derivatives scale with moment arm and velocity
    - _Requirements: 11.1, 11.2, 11.3, 11.4, 11.5, 11.6, 11.7, 11.9, 11.10, 12.1, 12.2, 12.3, 12.4, 12.5, 12.7, 12.8_
  
  - [ ]* 16.2 Write property test for static derivatives calculation
    - **Property 44: Static Derivatives Calculation**
    - **Validates: Requirements 11.1, 11.2, 11.3, 11.4, 11.5, 11.6, 11.7, 11.9, 11.10**
  
  - [ ]* 16.3 Write property test for dynamic derivatives calculation
    - **Property 45: Dynamic Derivatives Calculation**
    - **Validates: Requirements 12.1, 12.2, 12.3, 12.4, 12.5, 12.7, 12.8**

- [ ] 17. Implement physical bounds tests
  - [ ] 17.1 Create property/test_bounds.cpp with physical bounds tests
    - Test drag coefficient positivity
    - Test lift-to-drag ratio bounds
    - Test center of pressure bounds
    - Test static margin bounds
    - Test stall factor bounds
    - Test interference factor positivity
    - _Requirements: 13.1, 13.2, 13.3, 13.4, 13.5, 13.7_
  
  - [ ]* 17.2 Write property test for drag coefficient positivity
    - **Property 46: Drag Coefficient Positivity**
    - **Validates: Requirements 13.1**
  
  - [ ]* 17.3 Write property test for lift-to-drag ratio bounds
    - **Property 47: Lift-to-Drag Ratio Bounds**
    - **Validates: Requirements 13.2**
  
  - [ ]* 17.4 Write property test for center of pressure bounds
    - **Property 48: Center of Pressure Bounds**
    - **Validates: Requirements 13.3**
  
  - [ ]* 17.5 Write property test for static margin bounds
    - **Property 49: Static Margin Bounds**
    - **Validates: Requirements 13.4**
  
  - [ ]* 17.6 Write property test for stall factor bounds
    - **Property 50: Stall Factor Bounds**
    - **Validates: Requirements 13.5**
  
  - [ ]* 17.7 Write property test for interference factor positivity
    - **Property 51: Interference Factor Positivity**
    - **Validates: Requirements 13.7**

- [ ] 18. Implement symmetry property tests
  - [ ] 18.1 Create property/test_symmetry.cpp with symmetry tests
    - Test angle of attack symmetry (Cz and my negation)
    - Test sideslip angle symmetry (Cy and mz negation)
    - Test zero angle symmetry (Cy and Cz near zero)
    - Test control deflection symmetry
    - _Requirements: 14.1, 14.2, 14.3, 14.4, 14.5, 14.6, 14.7_
  
  - [ ]* 18.2 Write property test for angle of attack symmetry
    - **Property 52: Angle of Attack Symmetry**
    - **Validates: Requirements 14.1, 14.2**
  
  - [ ]* 18.3 Write property test for sideslip angle symmetry
    - **Property 53: Sideslip Angle Symmetry**
    - **Validates: Requirements 14.3, 14.4**
  
  - [ ]* 18.4 Write property test for zero angle symmetry
    - **Property 54: Zero Angle Symmetry**
    - **Validates: Requirements 14.5, 14.6**
  
  - [ ]* 18.5 Write property test for control deflection symmetry
    - **Property 55: Control Deflection Symmetry**
    - **Validates: Requirements 14.7**

- [ ] 19. Implement continuity property tests
  - [ ] 19.1 Create property/test_continuity.cpp with continuity tests
    - Test Mach number continuity across all regimes
    - Test angle of attack continuity including stall
    - Test derivative continuity
    - _Requirements: 15.1, 15.2, 15.3, 15.5, 15.6, 15.7_
  
  - [ ]* 19.2 Write property test for Mach number continuity
    - **Property 56: Mach Number Continuity**
    - **Validates: Requirements 15.1, 15.3, 15.6**
  
  - [ ]* 19.3 Write property test for angle of attack continuity
    - **Property 57: Angle of Attack Continuity**
    - **Validates: Requirements 15.2, 15.5**
  
  - [ ]* 19.4 Write property test for derivative continuity
    - **Property 58: Derivative Continuity**
    - **Validates: Requirements 15.7**

- [ ] 20. Implement force conversion tests
  - [ ] 20.1 Create unit/test_force_conversion.cpp with conversion tests
    - Test dimensional force calculation formula
    - Test dimensional moment calculation formula
    - Test dynamic pressure calculation
    - Test reference length usage
    - _Requirements: 17.1, 17.2, 17.3, 17.4, 17.5, 17.6_
  
  - [ ]* 20.2 Write property test for dimensional force conversion
    - **Property 59: Dimensional Force Conversion**
    - **Validates: Requirements 17.1, 17.2, 17.3, 17.4, 17.5, 17.6**

- [ ] 21. Checkpoint - Ensure property tests pass
  - Ensure all tests pass, ask the user if questions arise.

- [ ] 22. Implement reference data validation tests
  - [ ] 22.1 Create reference/reference_data/ directory and download reference data
    - Download NACA 0012 lift data from NASA Langley Research Center
    - Create naca0012_lift.csv with lift coefficient vs angle of attack
    - Create body_drag.csv with drag coefficients for standard nose shapes
    - Create transonic_drag.csv with drag rise data
    - Document sources and conditions in CSV comments
    - _Requirements: 19.6_
  
  - [ ] 22.2 Create reference/test_validation.cpp with validation tests
    - Test wing lift slope matches NACA 0012 data (within 10%)
    - Test body drag coefficients match published data (within 15%)
    - Test transonic drag rise trend matches experimental data
    - Test supersonic lift slopes match linearized theory (within 20%)
    - _Requirements: 19.1, 19.2, 19.3, 19.4, 19.7_

- [ ] 23. Implement integration and regression tests
  - [ ] 23.1 Create integration/test_pipeline.cpp with end-to-end tests
    - Test complete pipeline: JSON file → parse → validate → calculate → output
    - Test rocket configuration (body + fins)
    - Test aircraft configuration (body + wing + fins)
    - Test simple body configuration
    - _Requirements: 20.3_
  
  - [ ] 23.2 Create integration/test_regression.cpp with regression tests
    - Create regression test cases for subsonic flight (M=0.3, alpha=0)
    - Create regression test cases for transonic flight (M=1.0, alpha=5)
    - Create regression test cases for supersonic flight (M=2.0, alpha=10)
    - Create regression test cases for hypersonic flight (M=4.0, alpha=15)
    - Create regression test cases for high angle of attack (alpha=30)
    - Create regression test cases for sideslip (beta=10)
    - Store expected outputs in test file
    - _Requirements: 20.1, 20.2, 20.5, 20.6, 20.7_

- [ ] 24. Implement edge case tests
  - [ ] 24.1 Create unit/test_edge_cases.cpp with edge case tests
    - Test Mach = 1.0 (exact transonic) handles smoothly
    - Test alpha = 90° throws RangeError
    - Test very small Reynolds number clamping
    - Test very small aspect ratio handling
    - Test zero component area handling
    - Test zero total normal force center of pressure calculation
    - _Requirements: 16.3, 16.4, 16.5, 16.6, 16.7, 16.8_

- [ ] 25. Final checkpoint - Run complete test suite
  - Run all unit tests and verify pass
  - Run all property tests (100 iterations each) and verify pass
  - Run all integration tests and verify pass
  - Run all reference validation tests and verify pass
  - Generate coverage report and verify >90% line coverage
  - Document any test failures or coverage gaps
  - Ensure all tests pass, ask the user if questions arise.

- [ ] 26. Create test documentation
  - [ ] 26.1 Create tests/README.md with test suite documentation
    - Document test organization and structure
    - Document how to run tests (CMake commands, Catch2 options)
    - Document how to run specific test subsets using tags
    - Document property test configuration (iterations, seeds)
    - Document reference data sources and citations
    - Document expected test runtime
    - Document coverage goals and how to generate coverage reports
    - _Requirements: 19.6, 19.8_

## Notes

- Tasks marked with `*` are optional property-based tests that can be skipped for faster MVP
- Each property test should run minimum 100 iterations using Catch2 GENERATE
- All property tests should include comment tags referencing design property numbers
- Unit tests focus on specific examples, edge cases, and error handling
- Property tests focus on universal properties across randomly generated inputs
- Reference validation tests may take longer due to data loading and interpolation
- Total test suite runtime expected: ~10 minutes
- Coverage goal: >90% line coverage, >85% branch coverage
