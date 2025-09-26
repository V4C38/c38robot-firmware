# C38 Robot Controller Interface

A Next.js-based control interface for the C38 robot arm, providing a modern web-based alternative to the Qt C++ application.

## Features

- **Serial Communication**: Direct serial port communication with the robot arm
- **Tab-based Interface**: Organized controls in three main tabs:
  - Serial Port: Connect/disconnect to robot via serial port
  - Axis Control: Individual axis control with sliders (0-5 axes)
  - Testing: Run predefined test sequences and diagnostics
- **Real-time State Management**: Live updates of robot arm state
- **Command Configuration**: JSON-based command definitions

## Project Structure

```
controller-interface/
├── app/                    # Next.js app directory
├── components/            # React components
│   ├── RobotControlInterface.tsx  # Main interface component
│   └── tabs/             # Tab components
│       ├── SerialPortTab.tsx
│       ├── AxisControlTab.tsx
│       └── TestingTab.tsx
├── contexts/             # React contexts
│   └── RobotContext.tsx  # Global state management
├── lib/                  # Core libraries
│   └── serial/          # Serial communication
│       ├── SerialInterface.ts
│       └── serialInstance.ts
├── types/               # TypeScript type definitions
│   ├── arm.types.ts
│   └── command.types.ts
└── public/
    └── CommandConfig.json  # Command definitions

```

## Getting Started

### Prerequisites

- Node.js 18+ (Note: Some packages require Node.js 20+)
- npm or yarn
- Serial port access permissions

### Installation

```bash
cd controller-interface
npm install
```

### Development

```bash
npm run dev
```

Open [http://localhost:3000](http://localhost:3000) in your browser.

### Building for Production

```bash
npm run build
npm start
```

## Usage

1. **Connect to Robot**:
   - Navigate to the Serial Port tab
   - Select your robot's serial port from the dropdown
   - Click Connect

2. **Control Axes**:
   - Switch to the Axis Control tab
   - Use sliders to set target angles for each axis
   - Click "Set Angle" for individual axes or "Set All Angles"

3. **Run Tests**:
   - Go to the Testing tab
   - Select a test sequence from the dropdown
   - Click "Run Test"

## Command Structure

Commands are defined in `public/CommandConfig.json`. The interface supports:

- `homingSequence`: Home individual axes
- `setAxisAngle`: Set specific angle for an axis
- `getState`: Retrieve current robot state
- `emergencyStop`: Emergency stop all movement
- `setArmState`: Set complete arm state
- `runTest`: Execute predefined test sequences

## Serial Communication

The interface communicates using JSON messages over serial:

```typescript
// Command format
{
  "type": "command",
  "uuid": "unique-id",
  "command": "setAxisAngle",
  "axis": 0,
  "angle": 45.5
}

// Response format
{
  "type": "response",
  "uuid": "matching-uuid",
  "command": "setAxisAngle",
  "status": "success",
  "stateUpdate": {
    "axes": { "0": 45.5 }
  }
}
```

## Future Enhancements

- Three.js 3D visualization panel (right side)
- Enhanced diagnostics and logging
- Motion planning and trajectory visualization
- Multi-robot support
- WebSocket support for real-time updates

## Troubleshooting

### Serial Port Not Detected
- Ensure the robot is powered on and connected
- Check USB permissions (may require sudo on Linux)
- Try refreshing the port list

### Connection Issues
- Verify baud rate (default: 115200)
- Check that no other application is using the port
- Ensure proper USB drivers are installed

### Node Version Warnings
Some serial port packages require Node.js 20+. The app will work with warnings on Node.js 18, but consider upgrading for full compatibility.