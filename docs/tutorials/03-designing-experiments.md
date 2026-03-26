# Tutorial 3: Designing Experiments

Learn how to create experiment protocols using the visual block designer.

## Objectives

- Navigate the experiment designer
- Use core blocks (events, wizard actions, control flow)
- Build a branching experiment protocol

## What is an Experiment?

An **Experiment** defines the protocol for your study:

```
Experiment
├── Steps (ordered sequence)
│   ├── Actions (robot behaviors)
│   ├── Wizard Blocks (human decisions)
│   └── Control Flow (loops, branches)
├── Robot Actions (from plugins)
└── Parameters (configurable values)
```

## Step 1: Create an Experiment

1. Open your study
2. Go to **Experiments** tab
3. Click **New Experiment**

### Experiment Settings

| Field | Description |
|-------|-------------|
| Name | Protocol title |
| Description | What the experiment measures |
| Robot | Which robot to use |
| Version | Track protocol versions |

## Step 2: The Experiment Designer Interface

The designer has three main areas:

```
┌──────────────────────────────────────────────────────────────┐
│  Experiment: Robot Trust Study v1                    [Save]  │
├────────────┬─────────────────────────────────────────────────┤
│            │                                                  │
│  Blocks    │           Canvas                                 │
│  Library   │                                                  │
│            │    ┌─────────┐    ┌─────────┐                   │
│  ┌──────┐  │    │ Step 1  │───▶│ Step 2  │                   │
│  │Events│  │    │ Hook    │    │ Story   │                   │
│  ├──────┤  │    └─────────┘    └────┬────┘                   │
│  │Wizard│  │                        │                        │
│  ├──────┤  │                   ┌────▼────┐                   │
│  │Control│  │                   │ Step 3  │                   │
│  ├──────┤  │                   │ Check   │                   │
│  │Robot │  │                   └────┬────┘                   │
│  └──────┘  │                   ┌────┴────┐                   │
│            │              ┌────┴───┐ ┌───┴────┐              │
│            │              │Step 4a │ │Step 4b │              │
│            │              │Correct │ │ Wrong  │              │
│            │              └───┬────┘ └───┬────┘              │
│            │                  └─────┬─────┘                   │
│            │                   ┌────▼────┐                    │
│            │                   │ Step 5  │                    │
│            │                   │Conclude │                    │
│            │                   └─────────┘                    │
├────────────┴─────────────────────────────────────────────────┤
│  Properties Panel                                           │
│  ┌─────────────────────────────────────────────────────────┐ │
│  │ Step 1: The Hook                                       │ │
│  │ Duration: 25 seconds                                   │ │
│  │ Actions: 2 blocks                                      │ │
│  └─────────────────────────────────────────────────────────┘ │
└──────────────────────────────────────────────────────────────┘
```

## Step 3: Understanding Block Categories

### Events (Triggers)

Start your experiment with these blocks:

| Block | Description |
|-------|-------------|
| **Trial Start** | Triggers when trial begins |
| **Wizard Button** | Waits for wizard to press a button |
| **Timer** | Waits for a specified duration |
| **Participant Response** | Waits for participant input |

### Wizard Actions

Blocks the wizard can control:

| Block | Description |
|-------|-------------|
| **Say Text** | Robot speaks text |
| **Play Animation** | Play a predefined animation |
| **Show Image** | Display image on robot screen |
| **Move Robot** | Move robot to position |

### Control Flow

Control experiment progression:

| Block | Description |
|-------|-------------|
| **Branch** | Split into multiple paths |
| **Loop** | Repeat a sequence |
| **Wait** | Pause for duration |
| **Converge** | Merge multiple paths back |

### Robot Actions

Actions from your installed robot plugin:

| Block | Description |
|-------|-------------|
| **say_text** | Robot speaks |
| **walk_forward** | Robot walks forward |
| **turn_left** | Robot turns |
| **wave** | Robot waves |

## Step 4: Building "The Interactive Storyteller"

Let's build a simple storytelling experiment with branching:

### Step 1: The Hook (Start)

1. Click **+ Add Step**
2. Name it "The Hook"
3. Set type to **Robot**
4. Drag **Say Text** block:
   ```
   text: "Hello! I have a story to tell you. Are you ready?"
   ```
5. Drag **Move Arm** block:
   ```
   arm: right
   gesture: welcome
   ```

### Step 2: The Narrative

1. Add new step "The Narrative"
2. Connect from Step 1
3. Add **Say Text**:
   ```
   text: "Once upon a time, a traveler flew to Mars..."
   ```
4. Add **Turn Head** for gaze behavior:
   ```
   yaw: 1.5
   pitch: 0.0
   ```

### Step 3: Comprehension Check (Branching)

1. Add new step "Comprehension Check"
2. Set type to **Conditional**
3. Add **Ask Question**:
   ```
   question: "What color was the rock?"
   options:
     - Correct: "Red"
     - Incorrect: "Blue"
   ```
4. This creates two paths automatically

### Step 4: Branch Paths

**Branch A (Correct):**
```
Say: "Yes! It was a glowing red rock."
Emotion: Happy
```

**Branch B (Incorrect):**
```
Say: "Actually, it was red."
Emotion: Sad
```

### Step 5: Converge

1. Add new step "Story Continues"
2. Set type to **Converge**
3. Connect both branches to this step
4. Add concluding speech

### Step 6: Conclusion

1. Add final step "Conclusion"
2. Add **Say Text**: "The End. Thank you for listening!"
3. Add **Bow** animation

## Step 5: Block Properties

Each block has configurable properties:

### Say Text Block

```json
{
  "text": "Hello, how are you?",
  "language": "en-US",
  "speed": 1.0,
  "emotion": "neutral"
}
```

### Branch Block

```json
{
  "variable": "last_response",
  "options": [
    { "label": "Yes", "value": "yes", "nextStepId": "step_abc" },
    { "label": "No", "value": "no", "nextStepId": "step_xyz" }
  ]
}
```

### Loop Block

```json
{
  "iterations": 3,
  "maxDuration": 60,
  "children": [...]
}
```

## Step 6: Testing Your Experiment

### Preview Mode

Test your experiment without running a real trial:

1. Click **Preview** button
2. Step through each block
3. See timing and flow
4. Test branching decisions

### Simulation Mode

Run with a simulated robot:

1. Enable `NEXT_PUBLIC_SIMULATION_MODE=true`
2. Start a trial
3. Robot actions are logged but not executed
4. Great for protocol testing

## Advanced: Parallel Execution

Run multiple actions simultaneously:

```
Step: Greeting
├── Parallel Block
│   ├── Say: "Hello!"
│   ├── Move Arm: Wave
│   └── Move Head: Look at participant
```

## Experiment Versioning

Track protocol changes:

1. **Draft** - Experiment being designed
2. **Testing** - Being tested with participants
3. **Ready** - Approved for data collection
4. **Deprecated** - Superseded by newer version

## Common Patterns

### Linear Protocol

```
Start → Step 1 → Step 2 → Step 3 → End
```

### Branching Protocol

```
Start → Step 1
          ├── Condition A → Step 2a
          └── Condition B → Step 2b
```

### Loop Protocol

```
Start → Step 1 → Loop (3x) → Step 2 → End
             ↑
             └── (back to Step 1)
```

### Parallel Protocol

```
Start → Parallel
          ├── Action A
          ├── Action B
          └── Action C
        → Continue
```

## Troubleshooting

### Block Not Connecting

- Check step types are compatible
- Ensure no circular dependencies
- Verify conditions are complete

### Robot Action Not Available

- Install the robot plugin
- Check plugin is enabled for study
- Verify robot is connected

### Timing Issues

- Adjust duration estimates
- Use explicit wait blocks
- Test with real timing

## Next Steps

Now that you've designed your experiment:

1. **[Running Trials](04-running-trials.md)** - Execute your experiment
2. **[Wizard Interface](05-wizard-interface.md)** - Learn real-time control
3. **[Robot Integration](06-robot-integration.md)** - Connect your robot

---

**Previous**: [Your First Study](02-your-first-study.md) | **Next**: [Running Trials](04-running-trials.md)
