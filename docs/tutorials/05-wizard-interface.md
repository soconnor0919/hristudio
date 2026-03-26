# Tutorial 5: Wizard Interface

Learn how to use the real-time wizard control interface for Wizard-of-Oz studies.

## Objectives

- Navigate the wizard interface
- Control robot actions in real-time
- Make branching decisions
- Handle trial interruptions

## What is the Wizard Interface?

The **Wizard Interface** is your control center during trials. It provides:

- Real-time trial monitoring
- Robot action controls
- Decision-making tools
- Intervention capabilities
- Event logging

```
┌──────────────────────────────────────────────────────────────┐
│                    WIZARD INTERFACE                         │
├────────────────┬─────────────────────┬──────────────────────┤
│                │                     │                      │
│  Trial         │   Timeline          │   Robot              │
│  Controls      │   Progress          │   Status             │
│                │                     │                      │
│  ┌──────────┐  │  ┌───────────────┐  │  ┌────────────────┐  │
│  │ ▶ Play  │  │  │ 1 → 2 → 3 →  │  │  │ ● Connected    │  │
│  │ ⏸ Pause │  │  │         ↑     │  │  │ Battery: 85%   │  │
│  │ ⏹ Stop  │  │  │     Step 2    │  │  │ Position: (0,0)│  │
│  └──────────┘  │  └───────────────┘  │  └────────────────┘  │
│                │                     │                      │
│  ┌──────────┐  │  Progress: 40%      │  ┌────────────────┐  │
│  │ 📝 Notes │  │  Time: 00:05:23    │  │ Action Panel   │  │
│  │ ⚠ Alert │  │                     │  │                │  │
│  └──────────┘  │                     │  │ [Say Text]     │  │
│                │                     │  │ [Move Robot]   │  │
│                │                     │  │ [Wave]         │  │
│                │                     │  │ [Custom...]    │  │
│                │                     │  └────────────────┘  │
└────────────────┴─────────────────────┴──────────────────────┘
```

## Step 1: Accessing the Wizard Interface

### Method 1: From Trials List

1. Go to **Trials** in sidebar
2. Find your scheduled trial
3. Click **Open Wizard**

### Method 2: Direct URL

```
/trials/{trialId}/wizard
```

### Method 3: Trial Queue

1. Go to **Wizard Queue**
2. See all pending trials
3. Click **Start** on any trial

## Step 2: Understanding the Layout

### Left Panel: Trial Controls

| Control | Function |
|---------|----------|
| Play/Pause | Start or pause trial |
| Stop | End trial early |
| Notes | Add timestamped observations |
| Alert | Send alert to researchers |

### Center Panel: Timeline

- **Visual Progress**: See step progression
- **Current Position**: Highlighted current step
- **Navigation**: Click to jump to step (if allowed)
- **Time Display**: Elapsed and estimated remaining

### Right Panel: Robot Control

**Status Section:**
- Connection indicator
- Battery level
- Position tracking
- Sensor readings

**Action Section:**
- Quick action buttons
- Custom action builder
- Action history

## Step 3: Controlling the Robot

### Quick Actions

Pre-configured robot actions:

| Action | Description |
|--------|-------------|
| Say Text | Make robot speak |
| Wave | Wave gesture |
| Look at Me | Turn head toward participant |
| Look Away | Turn head elsewhere |
| Nod | Confirmation nod |
| Shake Head | Negation shake |

### Custom Say Text

1. Click **Say Text**
2. Enter text in popup:
   ```
   "Hello! Nice to meet you."
   ```
3. Select options:
   - Speed: Normal / Slow / Fast
   - Emotion: Neutral / Happy / Excited
4. Click **Execute**
5. Robot speaks the text

### Move Robot

1. Click **Move Robot**
2. Select movement type:
   - Walk Forward/Back
   - Turn Left/Right
   - Move Head
   - Move Arm
3. Set parameters
4. Execute

### Custom Actions

For advanced control:

1. Click **Custom...**
2. Select action from plugin
3. Configure parameters
4. Execute

## Step 4: Making Decisions

When the experiment reaches a branching point:

### Decision Popup

A popup appears with options:

```
┌─────────────────────────────────────────────────────────────┐
│ Branch Decision Required                                    │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  Step: Comprehension Check                                 │
│  Question: "What color was the rock?"                       │
│                                                             │
│  Participant's response: They said "blue" (incorrect)      │
│                                                             │
│  ┌─────────────────────────────────────────────────────┐    │
│  │  ○ Correct Response (Red)                         │    │
│  │    → Robot celebrates                             │    │
│  │                                                    │    │
│  │  ● Incorrect Response (Other)                     │    │
│  │    → Robot gently corrects                        │    │
│  └─────────────────────────────────────────────────────┘    │
│                                                             │
│  [Cancel]                               [Confirm Selection] │
└─────────────────────────────────────────────────────────────┘
```

### Decision Guidelines

1. **Observe** participant's actual response
2. **Consider** protocol criteria
3. **Select** appropriate branch
4. **Confirm** selection

### After Selection

- Decision is logged with timestamp
- Trial continues on selected path
- Both participant and robot continue

## Step 5: Handling Interruptions

### Pause Trial

When you need to pause:

1. Click **Pause** button
2. Optionally add reason:
   - Participant needs break
   - Technical issue
   - External interruption
3. Trial pauses, robot holds position

### Resume Trial

1. Click **Play** button
2. Trial resumes from pause point
3. Pause duration is logged

### Stop Trial

For early termination:

1. Click **Stop** button
2. Select reason:
   - Participant fatigue
   - Technical failure
   - Protocol deviation
   - Participant withdrawal
3. Confirm stop
4. Partial data is saved

### Add Notes

Record observations:

1. Click **Notes** button
2. Enter observation:
   ```
   Participant laughed at the robot's gesture.
   ```
3. Note is timestamped automatically
4. Notes appear in event log

### Send Alert

Notify researchers:

1. Click **Alert** button
2. Select alert type:
   - Technical issue
   - Safety concern
   - Protocol question
   - Other
3. Add description
4. Send alert

## Step 6: Monitoring Robot Status

### Connection Status

| Status | Icon | Meaning |
|--------|------|---------|
| Connected | ● Green | Robot responding |
| Connecting | ● Yellow | Attempting connection |
| Disconnected | ● Red | No robot connection |
| Error | ⚠ Orange | Connection error |

### Battery Monitor

View battery level:
- Green: > 50%
- Yellow: 20-50%
- Red: < 20%

### Sensor Display

Real-time sensor readings:
- Joint positions
- Touch sensors
- Sonar distances
- Camera feed (if available)

### Action Queue

See pending/executing actions:
```
Executing: Say Text "Hello!"
Pending: Move Head (queued)
```

## Step 7: Keyboard Shortcuts

Speed up your workflow:

| Key | Action |
|-----|--------|
| Space | Play/Pause toggle |
| Escape | Stop trial |
| N | Add note |
| A | Send alert |
| 1-9 | Execute quick action |
| ← → | Navigate timeline |
| ↑ ↓ | Select branch option |

## Step 8: Event Logging

All actions are logged automatically:

```
[14:32:05] Trial started
[14:32:07] Step 1: The Hook
[14:32:08] Action: Say Text "Hello!"
[14:32:11] Action: Move Arm Wave
[14:32:15] Step 2: The Narrative
[14:32:16] Action: Say Text "Once upon a time..."
[14:33:05] Step 3: Comprehension Check
[14:33:06] Action: Say Text "What color was the rock?"
[14:33:28] Wizard Note: "Participant said blue"
[14:33:30] Branch: Incorrect selected
[14:33:31] Step 4b: Correction
[14:33:32] Action: Say Text "Actually, it was red."
[14:34:05] Trial completed
```

## Trial Modes

### Observer Mode

For observers (read-only):
- View trial progress
- See robot status
- Cannot execute actions
- Can add notes

### Active Wizard Mode

Full control:
- Execute actions
- Make decisions
- Pause/resume
- Add notes/alerts

### Training Mode

Practice without real data:
- Simulated robot
- No data saved
- Safe to experiment

## Best Practices

### Before Trial

- [ ] Review experiment protocol
- [ ] Test robot connection
- [ ] Familiarize with action panel
- [ ] Know decision criteria

### During Trial

- [ ] Stay focused on participant
- [ ] Make decisions based on observation
- [ ] Document notable events
- [ ] Keep action log clean

### After Trial

- [ ] Review event log
- [ ] Add final notes
- [ ] Confirm data saved
- [ ] Prepare for next trial

## Troubleshooting

### Robot Not Responding

1. Check connection indicator
2. Verify network
3. Check robot power
4. Restart connection

### Actions Not Executing

1. Check action queue
2. Verify parameters
3. Check robot state (not in rest mode)

### Decision Popup Not Appearing

1. Check if step has branches
2. Verify step type is "conditional"
3. Contact researcher

## Next Steps

Mastered the wizard interface?

1. **[Robot Integration](06-robot-integration.md)** - Deep dive into robot control
2. **[Data & Analysis](08-data-and-analysis.md)** - Review trial data
3. **[Simulation Mode](09-simulation-mode.md)** - Practice without a robot

---

**Previous**: [Running Trials](04-running-trials.md) | **Next**: [Robot Integration](06-robot-integration.md)
