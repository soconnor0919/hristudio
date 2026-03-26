# Tutorial 9: Simulation Mode

Learn how to test HRIStudio experiments without a physical robot.

## Objectives

- Enable simulation mode
- Use the mock robot server
- Test experiments end-to-end
- Practice trial execution

## Why Simulation Mode?

Simulation mode allows you to:

- **Test protocols** without a robot
- **Train wizards** before live sessions
- **Debug experiments** in development
- **Run pilots** without robot access
- **Develop** on any computer

## Understanding Simulation Options

HRIStudio offers two simulation approaches:

| Approach | Pros | Cons |
|----------|------|------|
| **Client-side** | No server needed, instant | Limited robot simulation |
| **Mock Server** | Full rosbridge protocol | Requires running server |

### Client-Side Simulation

Simulates robot locally in the browser:
- No network required
- Instant startup
- Basic action timing
- Fake sensor data

### Mock Server

Full WebSocket server simulating rosbridge:
- Complete protocol support
- Realistic timing
- Sensor data simulation
- Better for integration testing

## Step 1: Enable Client-Side Simulation

### Quick Start

1. Create or edit `hristudio/.env.local`
2. Add:
   ```bash
   NEXT_PUBLIC_SIMULATION_MODE=true
   ```
3. Restart the dev server:
   ```bash
   bun dev
   ```

### Verify Enabled

Look for the simulation indicator in the UI:

```
┌─────────────────────────────────────────────────────────────┐
│  Wizard Interface                    [🔵 SIMULATION MODE] │
├─────────────────────────────────────────────────────────────┤
```

### Features Available

In simulation mode:

- ✅ All robot actions execute (simulated timing)
- ✅ Speech actions show estimated duration
- ✅ Movement actions track position
- ✅ Sensor data is simulated
- ✅ Trial execution works normally
- ❌ Real robot not controlled
- ❌ Physical interactions not possible

## Step 2: Start Mock Robot Server

For more complete testing, use the mock server:

### Option 1: Standalone Server

```bash
cd hristudio/scripts/mock-robot
bun install
bun dev
```

Server starts on `ws://localhost:9090`

### Option 2: Docker

```bash
cd nao6-hristudio-integration
docker compose -f docker-compose.yml -f docker-compose.mock.yml --profile mock up -d
```

### Verify Server Running

```bash
# Check container
docker ps

# Should show:
# CONTAINER ID   IMAGE         STATUS
# abc123def456   hristudio-mock-robot   Up 2 minutes
```

## Step 3: Connect to Mock Server

1. Go to the **NAO Test Page**: `/nao-test`
2. Ensure `NEXT_PUBLIC_SIMULATION_MODE` is NOT set (or set to false)
3. Click **Connect**
4. You should see:
   ```
   Connected to rosbridge
   Subscribed to: /joint_states, /bumper, /sonar/left, ...
   ```

## Step 4: Test Robot Actions

### From NAO Test Page

1. **Speech Test**
   - Enter text: "Hello, this is a test"
   - Click **Say**
   - See simulated speech duration

2. **Movement Test**
   - Set walk speed: 0.1 m/s
   - Click **Walk Forward**
   - Watch position update

3. **Head Control**
   - Set yaw: 1.0, pitch: 0.0
   - Click **Move Head**
   - See joint angles update

### From Wizard Interface

1. Start a trial
2. Execute actions as normal
3. Actions are sent to mock server
4. Mock server responds with simulated data

## Step 5: Simulated Actions Reference

### Speech Actions

| Action | Simulation Behavior |
|--------|---------------------|
| `say_text` | Duration = 1.5s + 300ms × word_count |
| `say_with_emotion` | Duration = 1.5s + 300ms × word_count + emotion_overhead |
| `wave_goodbye` | Duration = 3.0s |

### Movement Actions

| Action | Simulation Behavior |
|--------|---------------------|
| `walk_forward` | Position updates over 500ms |
| `walk_backward` | Position updates over 500ms |
| `turn_left` | Angle decreases over 500ms |
| `turn_right` | Angle increases over 500ms |
| `stop` | Velocity set to 0 |

### Sensor Simulation

| Sensor | Simulated Value |
|--------|-----------------|
| Battery | 85% ± 2% variation |
| Joint States | Random positions ±0.1 rad |
| Bumper | False (no contact) |
| Sonar | 0.5-1.0m (random) |
| Touch | False (no touch) |

## Step 6: Testing Experiment Protocols

### Full Protocol Test

1. Enable simulation mode
2. Create or open experiment
3. Schedule trial
4. Start trial in wizard interface
5. Execute through all steps
6. Verify timing and flow

### Test Checklist

- [ ] All steps execute in order
- [ ] Branching decisions work
- [ ] Timing estimates are accurate
- [ ] Event log captures everything
- [ ] No errors or warnings
- [ ] Trial completes successfully

### Debug Mode

Enable verbose logging:

```bash
# In browser console, run:
localStorage.setItem('debug', 'true')

# Refresh page
# Now see detailed action logs in console
```

## Step 7: Training Wizards

Simulation mode is perfect for training:

### Training Scenario 1: Basic Operation

1. Enable simulation mode
2. Load simple experiment
3. Practice:
   - Starting/pausing trials
   - Executing quick actions
   - Adding notes

### Training Scenario 2: Decision Making

1. Load branching experiment
2. Practice:
   - Observing participant cues
   - Selecting appropriate branches
   - Documenting decisions

### Training Scenario 3: Handling Issues

1. Practice:
   - Pausing for breaks
   - Responding to alerts
   - Stopping trials early

## Step 8: Development Workflow

### TDD with Simulation

```
┌─────────────────────────────────────────────────────────────┐
│                    Development Cycle                         │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  1. Design experiment in UI                                 │
│         │                                                  │
│         ▼                                                  │
│  2. Enable simulation mode                                 │
│         │                                                  │
│         ▼                                                  │
│  3. Run test trial                                         │
│         │                                                  │
│         ▼                                                  │
│  4. Review event log                                       │
│         │                                                  │
│         ▼                                                  │
│  5. Fix issues found                                       │
│         │                                                  │
│         └────────────┐                                     │
│                      │                                     │
│                      └ (repeat)                            │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

### Testing Checklist

Before running real trials:

- [ ] Experiment works in simulation
- [ ] All actions execute correctly
- [ ] Timing is acceptable
- [ ] Branching works as expected
- [ ] Wizard notes function properly
- [ ] Data exports correctly

## Step 9: Transitioning to Real Robot

When ready to test with real robot:

### Step 1: Disable Simulation

Remove or set to false:
```bash
NEXT_PUBLIC_SIMULATION_MODE=false
```

### Step 2: Connect Robot

1. Start Docker services
2. Verify robot connection
3. Test with NAO Test Page

### Step 3: Run Comparison Trial

1. Run same experiment on real robot
2. Compare timing and behavior
3. Adjust parameters as needed

### Step 4: Document Differences

Note any protocol adjustments needed:
- Timing differences
- Action parameter changes
- Branch criteria updates

## Troubleshooting

### Simulation Actions Not Working

1. Check `NEXT_PUBLIC_SIMULATION_MODE=true` is set
2. Verify no errors in browser console
3. Try refreshing the page

### Mock Server Connection Failed

```bash
# Check if server is running
docker ps | grep mock

# Check server logs
docker compose logs mock_robot

# Restart if needed
docker compose restart mock_robot
```

### Actions Execute But Nothing Happens

1. Check WebSocket URL is correct
2. Verify port 9090 is not blocked
3. Try client-side simulation instead

## Comparison: Simulation vs Real

| Aspect | Simulation | Real Robot |
|--------|------------|------------|
| Setup time | 1 min | 30+ min |
| Availability | Always | Requires robot |
| Cost | Free | Robot access needed |
| Timing accuracy | Estimated | Actual |
| Physical interaction | ✗ | ✓ |
| Sensor accuracy | Fake | Real |
| Network dependent | No | Yes |

## Best Practices

### When to Use Simulation

- During experiment design
- While robot unavailable
- For wizard training
- For debugging protocols
- For quick iteration

### When to Use Real Robot

- Final protocol validation
- Timing accuracy critical
- Physical interaction matters
- Sensor data needed
- Pre-study pilot

### Transition Checklist

Before real trials:
- [ ] Protocol tested in simulation
- [ ] Timing verified
- [ ] Actions calibrated
- [ ] Wizard team trained
- [ ] Backup plan ready

## Next Steps

Now that you've mastered simulation:

1. **[Robot Integration](06-robot-integration.md)** - Connect real robot
2. **[Running Trials](04-running-trials.md)** - Execute live trials
3. **[Your First Study](02-your-first-study.md)** - Run complete study

---

**Previous**: [Data & Analysis](08-data-and-analysis.md) | **Back**: [Tutorials Overview](../tutorials/README.md)
