# Design Document

## Overview

Кроссплатформенное десктопное приложение для настройки сценариев полёта баллистических объектов, построенное на Python 3.10+ с использованием Tkinter. Приложение следует архитектурному паттерну MVC (Model-View-Controller) для обеспечения модульности и maintainability. Основная функциональность включает создание и редактирование конфигураций различных типов объектов, валидацию параметров, управление JSON файлами сценариев и запуск внешних C++ симуляций с поддержкой стохастического анализа.

## Architecture

### MVC Pattern Implementation

Приложение использует модифицированный MVC паттерн, адаптированный для Tkinter:

**Model Layer:**
- `ScenarioModel`: Управляет данными сценария и бизнес-логикой
- `ObjectModel`: Базовый класс для всех типов объектов
- `ValidationModel`: Централизованная валидация параметров
- `SimulationModel`: Управление запуском внешних процессов

**View Layer:**
- `MainWindow`: Главное окно приложения
- `ObjectEditorView`: Редакторы для различных типов объектов
- `ParameterFormView`: Переиспользуемые формы параметров
- `StatusView`: Отображение статуса симуляций

**Controller Layer:**
- `ApplicationController`: Главный контроллер приложения
- `ScenarioController`: Управление операциями со сценариями
- `SimulationController`: Контроль выполнения симуляций
- `ValidationController`: Координация валидации

### Threading Architecture

Для обеспечения отзывчивости интерфейса используется многопоточная архитектура:

- **Main Thread**: GUI операции и обработка событий
- **Worker Threads**: Выполнение симуляций и файловых операций
- **Thread Pool**: Управление параллельными симуляциями для стохастического анализа

## Components and Interfaces

### Core Components

#### ScenarioModel
```python
class ScenarioModel:
    def __init__(self):
        self.objects: List[ObjectModel] = []
        self.metadata: Dict[str, Any] = {}
        self.observers: List[Observer] = []
    
    def add_object(self, obj: ObjectModel) -> None
    def remove_object(self, obj_id: str) -> None
    def validate_scenario(self) -> ValidationResult
    def to_json(self) -> Dict[str, Any]
    def from_json(self, data: Dict[str, Any]) -> None
```

#### ObjectModel Hierarchy
```python
class ObjectModel(ABC):
    def __init__(self, obj_id: str, obj_type: str):
        self.id = obj_id
        self.type = obj_type
        self.parameters: Dict[str, Any] = {}
    
    @abstractmethod
    def validate(self) -> ValidationResult
    @abstractmethod
    def to_dict(self) -> Dict[str, Any]
    @abstractmethod
    def from_dict(self, data: Dict[str, Any]) -> None

class GuidedMissileModel(ObjectModel):
    # Sensors, autopilot, aerodynamics, seeker, guidance methods
    
class UnguidedMissileModel(ObjectModel):
    # Aerodynamics without sensors/autopilot
    
class SimpleObjectModel(ObjectModel):
    # Constant velocity parameters
    
class UAVModel(ObjectModel):
    # Similar to guided missile with UAV-specific parameters
```

#### ValidationModel
```python
class ValidationModel:
    def __init__(self):
        self.validators: Dict[str, Validator] = {}
        self.error_messages: Dict[str, str] = {}
    
    def register_validator(self, param_name: str, validator: Validator) -> None
    def validate_parameter(self, param_name: str, value: Any) -> ValidationResult
    def validate_object(self, obj: ObjectModel) -> ValidationResult
```

### User Interface Components

#### MainWindow
Главное окно приложения с меню, панелью инструментов и основной рабочей областью:

```python
class MainWindow(tk.Tk):
    def __init__(self, controller: ApplicationController):
        # Menu bar: File, Edit, Simulation, Help
        # Toolbar: New, Open, Save, Run Simulation
        # Main area: Object list + editor panels
        # Status bar: Validation status, simulation progress
```

#### ObjectEditorView
Модульные редакторы для каждого типа объекта с вкладочной организацией:

```python
class ObjectEditorView(ttk.Frame):
    def __init__(self, parent, obj_type: str):
        # Tabbed interface for parameter categories
        # Real-time validation feedback
        # Parameter-specific input widgets
```

#### ParameterFormView
Переиспользуемые компоненты для различных типов параметров:

```python
class ParameterFormView(ttk.Frame):
    def create_numeric_field(self, name: str, min_val: float, max_val: float)
    def create_choice_field(self, name: str, options: List[str])
    def create_vector_field(self, name: str, dimensions: int)
```

### External Process Management

#### SimulationManager
```python
class SimulationManager:
    def __init__(self, max_concurrent: int):
        self.thread_pool = ThreadPoolExecutor(max_workers=max_concurrent)
        self.active_simulations: Dict[str, subprocess.Popen] = {}
    
    def run_simulation(self, config: Dict[str, Any]) -> Future
    def run_stochastic_analysis(self, base_config: Dict, variations: List[Dict]) -> Future
    def cancel_simulation(self, sim_id: str) -> None
```

## Data Models

### JSON Schema Structure

#### Scenario Configuration
```json
{
  "metadata": {
    "name": "string",
    "description": "string",
    "created": "ISO8601 timestamp",
    "modified": "ISO8601 timestamp"
  },
  "objects": [
    {
      "id": "string",
      "type": "guided_missile|unguided_missile|simple_object|uav",
      "parameters": {
        // Type-specific parameters
      }
    }
  ],
  "simulation_settings": {
    "time_step": "number",
    "duration": "number",
    "output_format": "string"
  }
}
```

#### Object Type Schemas

**Guided Missile:**
```json
{
  "sensors": {
    "accelerometer": {"noise_level": "number", "bias": "number"},
    "gyroscope": {"noise_level": "number", "drift": "number"},
    "gps": {"accuracy": "number", "update_rate": "number"}
  },
  "autopilot": {
    "pid_gains": {"kp": "number", "ki": "number", "kd": "number"},
    "control_limits": {"max_deflection": "number", "rate_limit": "number"}
  },
  "aerodynamics": {
    "drag_coefficient": "number",
    "lift_coefficient": "number",
    "reference_area": "number"
  },
  "seeker": {
    "type": "infrared|radar|optical",
    "field_of_view": "number",
    "range": "number"
  },
  "guidance": {
    "method": "proportional_navigation|pursuit|intercept",
    "navigation_constant": "number"
  }
}
```

**Unguided Missile:**
```json
{
  "aerodynamics": {
    "type": "full|simplified",
    "drag_coefficient": "number",
    "reference_area": "number"
  },
  "initial_conditions": {
    "velocity": {"x": "number", "y": "number", "z": "number"},
    "position": {"x": "number", "y": "number", "z": "number"}
  }
}
```

**Simple Object:**
```json
{
  "velocity": {"x": "number", "y": "number", "z": "number"},
  "initial_position": {"x": "number", "y": "number", "z": "number"},
  "mass": "number"
}
```

**UAV:**
```json
{
  "sensors": {
    // Similar to guided missile
  },
  "autopilot": {
    // UAV-specific autopilot parameters
    "flight_mode": "manual|stabilize|guided|auto",
    "waypoint_tolerance": "number"
  },
  "aerodynamics": {
    // UAV-specific aerodynamics
    "wing_span": "number",
    "aspect_ratio": "number"
  }
}
```

### Parameter Validation Rules

#### Numeric Parameters
- Range validation: min/max bounds
- Type validation: integer/float
- Unit consistency checks

#### Vector Parameters
- Dimension validation
- Magnitude constraints
- Coordinate system consistency

#### Enumerated Parameters
- Valid option checking
- Case-insensitive matching
- Default value handling

## Correctness Properties

*A property is a characteristic or behavior that should hold true across all valid executions of a system-essentially, a formal statement about what the system should do. Properties serve as the bridge between human-readable specifications and machine-verifiable correctness guarantees.*

### Property 1: Object Creation Completeness
*For any* object type (guided_missile, unguided_missile, simple_object, uav), when creating an object of that type, the resulting object should contain all required components specific to that type and no forbidden components.
**Validates: Requirements 1.1, 1.2, 1.3, 1.4**

### Property 2: Parameter Type Display Consistency  
*For any* object type selection, the displayed parameter fields should match exactly the parameter schema defined for that object type.
**Validates: Requirements 1.5**

### Property 3: Scenario Object Management
*For any* scenario and any object, adding the object to the scenario should result in the object being present in the scenario's object list, and removing it should result in its absence.
**Validates: Requirements 2.1, 2.3, 2.4**

### Property 4: Object Modification Persistence
*For any* object in a scenario and any valid parameter change, modifying the object's parameters should result in the changes being reflected in the scenario's object data.
**Validates: Requirements 2.5**

### Property 5: Parameter Validation Completeness
*For any* parameter and any input value, the validation result should correctly reflect whether the value meets the parameter's type, range, and requirement constraints.
**Validates: Requirements 3.1, 3.4, 3.5**

### Property 6: Validation Error Display
*For any* validation failure, the system should display appropriate error messages and visual indicators for the invalid fields.
**Validates: Requirements 3.2, 9.2**

### Property 7: Invalid Scenario Execution Prevention
*For any* scenario containing validation errors, attempting to execute simulation should be prevented and blocked.
**Validates: Requirements 3.3, 9.3**

### Property 8: JSON Serialization Round Trip
*For any* valid scenario, serializing to JSON and then deserializing should produce an equivalent scenario with all object parameters preserved.
**Validates: Requirements 4.1, 4.2, 4.4, 4.5**

### Property 9: JSON Export Compatibility
*For any* scenario exported to JSON, the resulting JSON structure should conform to the Flight_Simulator input format specification.
**Validates: Requirements 4.3**

### Property 10: External Process Management
*For any* valid scenario configuration, launching the Flight_Simulator should create an external process with the correct parameters and allow monitoring of its execution status.
**Validates: Requirements 5.1, 5.2**

### Property 11: Simulation Result Capture
*For any* completed simulation (successful or failed), the system should capture and display the appropriate result or error information from the Flight_Simulator process.
**Validates: Requirements 5.3, 5.4**

### Property 12: Simulation Cancellation
*For any* running simulation, the cancellation operation should successfully terminate the external process and update the UI status.
**Validates: Requirements 5.5**

### Property 13: Stochastic Parameter Variation
*For any* parameter configured with a probability distribution, the stochastic analysis should generate variations according to the specified distribution parameters.
**Validates: Requirements 6.1**

### Property 14: Parallel Execution Resource Management
*For any* stochastic analysis request, the number of concurrent simulations should not exceed the configured CPU core limit.
**Validates: Requirements 6.2, 6.3**

### Property 15: Stochastic Analysis Progress Tracking
*For any* running stochastic analysis, the system should provide accurate progress information and statistical result aggregation upon completion.
**Validates: Requirements 6.4, 6.5**

### Property 16: Cross-Platform Library Compatibility
*For any* imported Python library or Tkinter component used in the application, it should be available and functional on Windows, Linux, and macOS platforms.
**Validates: Requirements 8.4**

### Property 17: Platform-Independent File Operations
*For any* file path or system operation, the implementation should work correctly across different operating systems without modification.
**Validates: Requirements 8.5**

### Property 18: Error Message User-Friendliness
*For any* error condition, the displayed error message should be understandable to users and provide actionable information for resolution.
**Validates: Requirements 9.1**

### Property 19: Error Logging Completeness
*For any* error or exception, detailed information should be logged for debugging purposes while displaying user-friendly messages in the UI.
**Validates: Requirements 9.4**

### Property 20: Contextual Help Availability
*For any* parameter configuration field, contextual help information should be available and accessible to users.
**Validates: Requirements 9.5**

### Property 21: Validation Interface Consistency
*For any* parameter validation across different modules, the validation interface and behavior should be consistent and follow the same patterns.
**Validates: Requirements 10.3**

### Property 22: Modular Extensibility
*For any* new object type addition, the system should allow integration without requiring modifications to existing object type implementations.
**Validates: Requirements 10.5**

## Error Handling

### Validation Error Management
- **Real-time Validation**: Parameter validation occurs on field change events
- **Batch Validation**: Complete scenario validation before simulation execution
- **Error Aggregation**: Multiple validation errors are collected and displayed together
- **Error Recovery**: Clear guidance provided for resolving validation issues

### External Process Error Handling
- **Process Launch Failures**: Graceful handling of simulator startup errors
- **Runtime Errors**: Capture and display simulator error output
- **Process Termination**: Proper cleanup of terminated or cancelled processes
- **Resource Management**: Prevention of resource leaks from failed processes

### File Operation Error Handling
- **JSON Parsing Errors**: Detailed error messages for malformed JSON files
- **File Access Errors**: Handling of permission and file system issues
- **Data Corruption**: Validation of loaded data integrity
- **Backup and Recovery**: Automatic backup creation before file modifications

### UI Error Handling
- **Widget State Management**: Consistent UI state during error conditions
- **Error Message Display**: Non-blocking error dialogs and status indicators
- **Input Sanitization**: Prevention of invalid input from reaching business logic
- **Graceful Degradation**: Partial functionality when components fail

## Testing Strategy

### Dual Testing Approach

The application will use both unit testing and property-based testing for comprehensive coverage:

**Unit Tests:**
- Specific examples and edge cases for each object type
- Integration points between GUI components and business logic
- Error conditions and exception handling scenarios
- Platform-specific functionality verification

**Property Tests:**
- Universal properties that hold across all inputs using Hypothesis library
- Comprehensive input coverage through randomization
- Each property test configured to run minimum 100 iterations
- Properties tagged with references to design document properties

**Property-Based Testing Configuration:**
- Library: Hypothesis for Python property-based testing
- Test iterations: Minimum 100 per property test
- Tag format: **Feature: ballistic-flight-gui, Property {number}: {property_text}**
- Each correctness property implemented by a single property-based test

**Testing Coverage:**
- Unit tests focus on concrete examples, integration points, and error conditions
- Property tests verify general correctness across all possible inputs
- Together they provide comprehensive validation of both specific behaviors and universal properties

### Test Organization
- Separate test modules for each component (Model, View, Controller)
- Integration tests for end-to-end workflows
- Performance tests for stochastic analysis with multiple concurrent simulations
- Cross-platform compatibility tests for each supported operating system