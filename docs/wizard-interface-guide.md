# Wizard Interface Guide

## Overview

The HRIStudio wizard interface provides a comprehensive, real-time trial execution environment with a consolidated 3-panel design optimized for efficient experiment control and monitoring.

## Interface Layout

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                          Trial Execution Header                              │
│                    [Trial Name] - [Participant] - [Status]                  │
└─────────────────────────────────────────────────────────────────────────────┘
┌──────────────┬──────────────────────────────────────┬──────────────────────┐
│              │                                       │                      │
│ Trial        │   Execution Timeline                  │ Robot Control        │
│ Control      │                                       │ & Status             │
│              │                                       │                      │
│ ┌──────────┐ │ ┌──┬──┬──┬──┬──┐ Step Progress      │ 📷 Camera View       │
│ │ Start    │ │ │✓ │✓ │● │  │  │                    │                      │
│ │ Pause    │ │ └──┴──┴──┴──┴──┘                    │ Connection: ✓        │
│ │ Next Step│ │                                       │                      │
│ │ Complete │ │ Current Step: "Greeting"             │ Autonomous Life: ON  │
│ │ Abort    │ │ ┌────────────────────────────────┐   │                      │
│ └──────────┘ │ │ Actions:                       │   │ Robot Actions:       │
│              │ │ • Say "Hello"          [Run]   │   │ ┌──────────────────┐ │
│ Progress:    │ │ • Wave Hand            [Run]   │   │ │ Quick Commands   │ │
│ Step 3/5     │ │ • Wait 2s              [Run]   │   │ └──────────────────┘ │
│              │ └────────────────────────────────┘   │                      │
│              │                                       │ Movement Controls    │
│              │                                       │ Quick Actions        │
│              │                                       │ Status Monitoring    │
└──────────────┴──────────────────────────────────────┴──────────────────────┘
```

## Panel Descriptions

### Left Panel: Trial Control

**Purpose**: Manage overall trial flow and progression

**Features:**
- **Start Trial**: Begin experiment execution
- **Pause/Resume**: Temporarily halt trial without aborting
- **Next Step**: Manually advance to next step (when all actions complete)
- **Complete Trial**: Mark trial as successfully completed
- **Abort Trial**: Emergency stop with reason logging

**Progress Indicators:**
- Current step number (e.g., "Step 3 of 5")
- Overall trial status
- Time elapsed

**Best Practices:**
- Use Pause for participant breaks
- Use Abort only for unrecoverable issues
- Document abort reasons thoroughly

---

### Center Panel: Execution Timeline

**Purpose**: Visualize experiment flow and execute current step actions

#### Horizontal Step Progress Bar

**Features:**
- **Visual Overview**: See all steps at a glance
- **Step States**:
  - ✓ **Completed** (green checkmark, primary border)
  - ● **Current** (highlighted, ring effect)
  - ○ **Upcoming** (muted appearance)
- **Click Navigation**: Jump to any step (unless read-only)
- **Horizontal Scroll**: For experiments with many steps

**Step Card Elements:**
- Step number or checkmark icon
- Truncated step name (hover for full name)
- Visual state indicators

#### Current Step View

**Features:**
- **Step Header**: Name and description
- **Action List**: Vertical timeline of actions
- **Action States**:
  - Completed actions (checkmark)
  - Active action (highlighted, pulsing)
  - Pending actions (numbered)
- **Action Controls**: Run, Skip, Mark Complete buttons
- **Progress Tracking**: Auto-scrolls to active action

**Action Types:**
- **Wizard Actions**: Manual tasks for the wizard
- **Robot Actions**: Commands sent to the robot
- **Control Flow**: Loops, branches, parallel execution
- **Observations**: Data collection and recording

**Best Practices:**
- Review step description before starting
- Execute actions in order unless branching
- Use Skip sparingly and document reasons
- Verify robot action completion before proceeding

---

### Right Panel: Robot Control & Status

**Purpose**: Unified location for all robot-related controls and monitoring

#### Camera View
- Live video feed from robot or environment
- Multiple camera support (switchable)
- Full-screen mode available

#### Connection Status
- **ROS Bridge**: WebSocket connection state
- **Robot Status**: Online/offline indicator
- **Reconnect**: Manual reconnection button
- **Auto-reconnect**: Automatic retry on disconnect

#### Autonomous Life Toggle
- **Purpose**: Enable/disable robot's autonomous behaviors
- **States**:
  - ON: Robot exhibits idle animations, breathing, awareness
  - OFF: Robot remains still, fully manual control
- **Best Practice**: Turn OFF during precise interactions

#### Robot Actions Panel
- **Quick Commands**: Pre-configured robot actions
- **Parameter Controls**: Adjust action parameters
- **Execution Status**: Real-time feedback
- **Action History**: Recent commands log

#### Movement Controls
- **Directional Pad**: Manual robot navigation
- **Speed Control**: Adjust movement speed
- **Safety Limits**: Collision detection and boundaries
- **Emergency Stop**: Immediate halt

#### Quick Actions
- **Text-to-Speech**: Send custom speech commands
- **Preset Gestures**: Common robot gestures
- **LED Control**: Change robot LED colors
- **Posture Control**: Sit, stand, crouch commands

#### Status Monitoring
- **Battery Level**: Remaining charge percentage
- **Joint Status**: Motor temperatures and positions
- **Sensor Data**: Ultrasonic, tactile, IMU readings
- **Warnings**: Overheating, low battery, errors

**Best Practices:**
- Monitor battery level throughout trial
- Check connection status before robot actions
- Use Emergency Stop for safety concerns
- Document any robot malfunctions

---

## Workflow Guide

### Pre-Trial Setup

1. **Verify Robot Connection**
   - Check ROS Bridge status (green indicator)
   - Test robot responsiveness with quick action
   - Confirm camera feed is visible

2. **Review Experiment Protocol**
   - Scan horizontal step progress bar
   - Review first step's actions
   - Prepare any physical materials

3. **Configure Robot Settings** (Researchers/Admins only)
   - Click Settings icon in robot panel
   - Adjust speech, movement, connection parameters
   - Save configuration for this study

### During Trial Execution

1. **Start Trial**
   - Click "Start" in left panel
   - First step becomes active
   - First action highlights in timeline

2. **Execute Actions**
   - Follow action sequence in center panel
   - Use action controls (Run/Skip/Complete)
   - Monitor robot status in right panel
   - Document any deviations

3. **Navigate Steps**
   - Wait for "Complete Step" button after all actions
   - Click to advance to next step
   - Or click step in progress bar to jump

4. **Handle Issues**
   - **Participant Question**: Use Pause
   - **Robot Malfunction**: Check status panel, use Emergency Stop if needed
   - **Protocol Deviation**: Document in notes, continue or abort as appropriate

### Post-Trial Completion

1. **Complete Trial**
   - Click "Complete Trial" after final step
   - Confirm completion dialog
   - Trial marked as completed

2. **Review Data**
   - All actions logged with timestamps
   - Robot commands recorded
   - Sensor data captured
   - Video recordings saved

---

## Control Flow Features

### Loops

**Behavior:**
- Loops execute their child actions repeatedly
- **Implicit Approval**: Wizard automatically approves each iteration
- **Manual Override**: Wizard can skip or abort loop
- **Progress Tracking**: Shows current iteration (e.g., "2 of 5")

**Best Practices:**
- Monitor participant engagement during loops
- Use abort if participant shows distress
- Document any skipped iterations

### Branches

**Behavior:**
- Conditional execution based on criteria
- Wizard selects branch path
- Only selected branch actions execute
- Other branches are skipped

**Best Practices:**
- Review branch conditions before choosing
- Document branch selection rationale
- Ensure participant meets branch criteria

### Parallel Execution

**Behavior:**
- Multiple actions execute simultaneously
- All must complete before proceeding
- Independent progress tracking

**Best Practices:**
- Monitor all parallel actions
- Be prepared for simultaneous robot and wizard tasks
- Coordinate timing carefully

---

## Keyboard Shortcuts

| Shortcut | Action |
|----------|--------|
| `Space` | Start/Pause Trial |
| `→` | Next Step |
| `Esc` | Abort Trial (with confirmation) |
| `R` | Run Current Action |
| `S` | Skip Current Action |
| `C` | Complete Current Action |
| `E` | Emergency Stop Robot |

---

## Troubleshooting

### Robot Not Responding

1. Check ROS Bridge connection (right panel)
2. Click Reconnect button
3. Verify robot is powered on
4. Check network connectivity
5. Restart ROS Bridge if needed

### Camera Feed Not Showing

1. Verify camera is enabled in robot settings
2. Check camera topic in ROS
3. Refresh browser page
4. Check camera hardware connection

### Actions Not Progressing

1. Verify action has completed
2. Check for error messages
3. Manually mark complete if stuck
4. Document issue in trial notes

### Timeline Not Updating

1. Refresh browser page
2. Check WebSocket connection
3. Verify trial status is "in_progress"
4. Contact administrator if persists

---

## Role-Specific Features

### Wizards
- Full trial execution control
- Action execution and skipping
- Robot control (if permitted)
- Real-time decision making

### Researchers
- All wizard features
- Robot settings configuration
- Trial monitoring and oversight
- Protocol deviation approval

### Observers
- **Read-only access**
- View trial progress
- Monitor robot status
- Add annotations (no control)

### Administrators
- All features enabled
- System configuration
- Plugin management
- Emergency overrides

---

## Best Practices Summary

✅ **Before Trial**
- Verify all connections
- Test robot responsiveness
- Review protocol thoroughly

✅ **During Trial**
- Follow action sequence
- Monitor robot status continuously
- Document deviations immediately
- Use Pause for breaks, not Abort

✅ **After Trial**
- Complete trial properly
- Review captured data
- Document any issues
- Debrief with participant

❌ **Avoid**
- Skipping actions without documentation
- Ignoring robot warnings
- Aborting trials unnecessarily
- Deviating from protocol without approval

---

## Additional Resources

- **[Quick Reference](./quick-reference.md)** - Essential commands and shortcuts
- **[Implementation Details](./implementation-details.md)** - Technical architecture
- **[NAO6 Quick Reference](./nao6-quick-reference.md)** - Robot-specific commands
- **[Troubleshooting Guide](./nao6-integration-complete-guide.md)** - Detailed problem resolution