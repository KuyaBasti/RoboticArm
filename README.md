# Robotic Arm Drawing System

A C++ implementation of a 2-link planar robotic arm capable of interpreting G-code commands to perform precision drawing operations. This system combines inverse kinematics, numerical interpolation algorithms, and servo motor control to create a versatile drawing robot.

## 🤖 System Overview

This robotic arm system translates standard CNC G-code commands into precise motor movements, enabling the robot to draw complex shapes including lines, arcs, and circles. The system implements real-time inverse kinematics to convert Cartesian coordinates into joint angles for accurate positioning.

### Key Features

- **G-code Interpretation**: Supports standard CNC commands (G00, G01, G02, G03)
- **Inverse Kinematics**: Real-time calculation of joint angles from Cartesian coordinates
- **Smooth Interpolation**: Linear and circular arc interpolation with configurable step sizes
- **Serial Communication**: Direct motor control via RS-232 interface
- **Precision Control**: 0.12° angular resolution with incremental positioning

## 🏗️ System Architecture

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   G-code Input  │───▶│  Parser Module   │───▶│  Math Engine    │
│   (test*.txt)   │    │  (Parse.cpp)     │    │ (RinoMath.cpp)  │
└─────────────────┘    └──────────────────┘    └─────────────────┘
                                                         │
                                                         ▼
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│  Motor Control  │◀───│  Motion Planner  │◀───│ Coordinate      │
│(RhrinoSpecific) │    │ (MoveToPoint.cpp)│    │ Transformation  │
└─────────────────┘    └──────────────────┘    └─────────────────┘
```

### Hardware Configuration

- **E Motor**: Shoulder joint (base rotation)
- **D Motor**: Elbow joint (arm extension)
- **F Motor**: Tool control (pen up/down)
- **Arm Segments**: Two 9.0-unit links (configurable)
- **Communication**: RS-232 serial at 9600 baud

## 📐 Mathematical Foundations

### Inverse Kinematics

The system solves the inverse kinematics problem using trigonometric relationships:

**Elbow Angle (θ₂):**
```cpp
cos(θ₂) = (x² + y² - L₁² - L₂²) / (2·L₁·L₂)
```

**Shoulder Angle (θ₁):**
```cpp
θ₁ = arcsin(L₁·sin(θ₂)/r) + atan(y/x)
```

Where:
- `L₁`, `L₂` = arm segment lengths (default: 9.0 units)
- `x`, `y` = target Cartesian coordinates
- `r` = √(x² + y²)

### Interpolation Algorithms

#### Linear Interpolation (G01)
```cpp
x(t) = x₁ + t·cos(θ)
y(t) = y₁ + t·sin(θ)
```

#### Circular Interpolation (G02/G03)
```cpp
x(θ) = R·cos(θ) + Cₓ
y(θ) = R·sin(θ) + Cᵧ
```

## 🎯 Supported G-Code Commands

| Command | Description | Example |
|---------|-------------|---------|
| `G00` | Rapid positioning (pen up) | `G00X10.0Y5.0` |
| `G01` | Linear interpolation (drawing) | `G01X15.0Y10.0` |
| `G02` | Clockwise circular arc | `G02X8.0Y6.0I2.0J1.0` |
| `G03` | Counter-clockwise circular arc | `G03X5.0Y8.0I-1.0J2.0` |

### Parameter Format
- `X`, `Y`: Target coordinates
- `I`, `J`: Arc center offsets from start point
- Step resolution: 0.2 units (configurable)

## 🗂️ File Structure

```
RoboticArm/
├── Core Mathematics
│   ├── RinoMath.cpp          # Inverse kinematics engine
│   ├── RhinoMath.h           # Math function declarations
│   └── RhinoMan.h            # Angle pair structures
├── Motion Control
│   ├── MoveToPoint.cpp       # High-level motion commands
│   ├── MoveToPoint.h         # Motion interface
│   ├── RhrinoSpecific.cpp    # Low-level motor control
│   └── RhrinoSpecific.h      # Hardware abstraction
├── Path Planning
│   ├── Line.cpp              # Linear interpolation
│   ├── Line.h                # Line drawing interface
│   ├── CircleC.cpp           # Circular arc interpolation
│   └── CircleC.h             # Circle drawing interface
├── G-Code Processing
│   ├── Parse.cpp             # Command interpreter
│   └── Parse.h               # Parser interface
├── Communication
│   ├── tserial.cpp           # Serial communication
│   └── tserial.h             # Serial interface
├── Configuration
│   ├── MasterHeader.h        # Common data structures
│   └── test*.txt             # Sample G-code files
└── Project Files
    ├── ParseGCodeE4_27.sln   # Visual Studio solution
    └── ParseGCodeE4_27.vcxproj # Project configuration
```

## ⚙️ Technical Specifications

### Motion Parameters
- **Angular Resolution**: 0.12° per motor step
- **Linear Step Size**: 0.2 units (configurable)
- **Workspace**: Circular, radius = L₁ + L₂ (18.0 units default)
- **Joint Ranges**: Limited by physical constraints

### Communication Protocol
- **Interface**: RS-232 Serial
- **Baud Rate**: 9600
- **Data Format**: 8N1 (8 data bits, no parity, 1 stop bit)
- **Command Format**: `[Motor][Direction][Steps]` (e.g., "E+50")

### Performance Characteristics
- **Position Accuracy**: ±0.1 units
- **Repeatability**: ±0.05 units
- **Maximum Speed**: 50 steps per command cycle

## 🚀 Getting Started

### Prerequisites
- Windows development environment
- Visual Studio (C++ compiler)
- Serial port hardware interface
- Compatible servo motors

### Building the Project

1. **Clone the repository**
   ```bash
   git clone <repository-url>
   cd RoboticArm
   ```

2. **Open in Visual Studio**
   ```bash
   start ParseGCodeE4_27.sln
   ```

3. **Configure serial port**
   - Modify `COM1` in `RhrinoSpecific.h` if needed
   - Ensure correct baud rate (9600)

4. **Build the solution**
   - Build → Build Solution (Ctrl+Shift+B)

### Hardware Setup

1. **Connect Motors**
   - E Motor → Shoulder joint
   - D Motor → Elbow joint  
   - F Motor → Tool mechanism

2. **Serial Interface**
   - Connect controller to PC via RS-232
   - Verify COM port assignment

3. **Calibration**
   - Set home position (TickAng1 = TickAng2 = 750)
   - Verify arm lengths match configuration

## 🔧 Configuration

### Arm Geometry
Modify in `RhinoMath` constructor:
```cpp
RhinoMath robot(9.0, 9.0);  // L1, L2 in units
```

### Step Resolution
Adjust in motion functions:
```cpp
LineTest.getLineSteps(0.2);  // Linear step size
TestCircle.DoitC(..., 0.2, ...);  // Angular step size
```

### Motor Calibration
Update tick values in `RhrinoSpecific.h`:
```cpp
double TickAng1 = 750;  // E motor home position
double TickAng2 = 750;  // D motor home position
```

## 📊 Performance Analysis

### Workspace Limitations
- **Reachable Area**: Annular region between (L₁-L₂) and (L₁+L₂)
- **Singularities**: Full extension and full retraction
- **Dead Zones**: Areas behind the base joint

### Accuracy Factors
- **Step quantization**: ±0.06° per motor step
- **Mechanical backlash**: Variable with hardware
- **Thermal drift**: Minimal for short operations

## 🐛 Troubleshooting

### Common Issues

**Motor Not Responding**
- Verify COM port connection
- Check baud rate settings
- Ensure power supply adequate

**Inaccurate Positioning**
- Calibrate home positions
- Verify arm length parameters
- Check for mechanical binding

**G-Code Parse Errors**
- Validate command syntax
- Check for unsupported commands
- Verify coordinate ranges

### Development Guidelines
- Follow existing code style
- Add unit tests for new features
- Update documentation
- Test with hardware when possible

## 🔗 References

- **Robotics**: Craig, J.J. "Introduction to Robotics: Mechanics and Control"
- **CNC Programming**: Smid, P. "CNC Programming Handbook"
- **Inverse Kinematics**: Siciliano, B. "Robotics: Modelling, Planning and Control"

---

**Note**: This system is designed for educational purposes and may require hardware-specific modifications for production use. 