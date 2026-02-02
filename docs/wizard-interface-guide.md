# Wizard Interface Guide

## Overview

The Wizard Interface is a real-time control panel for conducting Human-Robot Interaction (HRI) trials. It provides wizards with comprehensive tools to execute experiment protocols, monitor participant interactions, and control robot behaviors in real-time.

## Key Features

- **Real-time Trial Execution**: Live step-by-step protocol execution with WebSocket connectivity
- **Robot Status Monitoring**: Battery levels, connection status, sensor readings, and position tracking
- **Participant Information**: Demographics, consent status, and session details
- **Live Event Logging**: Real-time capture of all trial events and wizard interventions
- **Action Controls**: Quick access to common wizard actions and robot commands

## WebSocket System

### Connection Setup

The wizard interface automatically connects to a WebSocket server for real-time communication:

```typescript
// WebSocket URL format
wss://your-domain.com/api/websocket?trialId={TRIAL_ID}&token={AUTH_TOKEN}
```

### Message Types

#### Incoming Messages (from server):
- `connection_established` - Connection acknowledgment
- `trial_status` - Current trial state and step information
- `trial_action_executed` - Confirmation of action execution
- `step_changed` - Step transition notifications
- `intervention_logged` - Wizard intervention confirmations

#### Outgoing Messages (to server):
- `heartbeat` - Keep connection alive
- `trial_action` - Execute trial actions (start, complete, abort)
- `wizard_intervention` - Log wizard interventions
- `step_transition` - Advance to next step

### Example Usage

```typescript
// Start a trial
webSocket.sendMessage({
  type: "trial_action",
  data: {
    actionType: "start_trial",
    step_index: 0,
    data: { notes: "Trial started by wizard" }
  }
});

// Log wizard intervention
webSocket.sendMessage({
  type: "wizard_intervention",
  data: {
    action_type: "manual_correction",
    step_index: currentStepIndex,
    action_data: { message: "Clarified instruction" }
  }
});
```

## Trial Execution Workflow

### 1. Pre-Trial Setup
- Verify participant consent and demographics
- Check robot connection and status
- Review experiment protocol steps
- Confirm WebSocket connectivity

### 2. Starting a Trial
1. Click "Start Trial" button
2. System automatically:
   - Updates trial status to "in_progress"
   - Records start timestamp
   - Loads first protocol step
   - Broadcasts status to all connected clients

### 3. Step-by-Step Execution
- **Current Step Display**: Shows active step details and actions
- **Execute Step**: Trigger step-specific actions (robot commands, wizard prompts)
- **Next Step**: Advance to subsequent protocol step
- **Quick Actions**: Access common wizard interventions

### 4. Real-time Monitoring
- **Robot Status**: Live updates on battery, signal, position, sensors
- **Event Log**: Chronological list of all trial events
- **Progress Tracking**: Visual progress bar and step completion status

### 5. Trial Completion
- Click "Complete" for successful trials
- Click "Abort" for early termination
- System records end timestamp and final status
- Automatic redirect to analysis page

## Experiment Data Integration

### Loading Real Experiment Steps

The wizard interface automatically loads experiment steps from the database:

```typescript
// Steps are fetched from the experiments API
const { data: experimentSteps } = api.experiments.getSteps.useQuery({
  experimentId: trial.experimentId
});
```

### Step Types and Actions

Supported step types from the experiment designer:
- **Wizard Steps**: Manual wizard actions and prompts
- **Robot Steps**: Automated robot behaviors and movements
- **Parallel Steps**: Concurrent actions executed simultaneously
- **Conditional Steps**: Branching logic based on participant responses

## Seed Data and Testing

### Available Test Data

The development database includes realistic test scenarios:

```bash
# Seed the database with test data
bun db:seed

# Default login credentials
Email: sean@soconnor.dev
Password: password123
```

### Test Experiments

1. **"Basic Interaction Protocol 1"** (Study: Real-time HRI Coordination)
   - 3 steps: Introduction, Wait for Response, Robot Feedback
   - Includes wizard actions and NAO robot integration
   - Estimated duration: 25 minutes

2. **"Dialogue Timing Pilot"** (Study: Wizard-of-Oz Dialogue Study)
   - Multi-step protocol with parallel and conditional actions
   - Timer-based transitions and conditional follow-ups
   - Estimated duration: 35 minutes

### Test Participants

Pre-loaded participants with complete demographics:
- Various age groups (18-65)
- Different educational backgrounds
- Robot experience levels
- Consent already verified

## Robot Integration

### Supported Robots

- **TurtleBot3 Burger**: Navigation and sensing capabilities
- **NAO Humanoid Robot**: Speech, gestures, and animations
- **Plugin System**: Extensible support for additional platforms

### Robot Actions

Common robot actions available during trials:
- **Speech**: Text-to-speech with configurable speed/volume
- **Movement**: Navigation commands and position control
- **Gestures**: Pre-defined animation sequences
- **LED Control**: Visual feedback through color changes
- **Sensor Readings**: Real-time environmental data

## Error Handling and Troubleshooting

### WebSocket Connection Issues

- **Connection Failed**: Check network connectivity and server status
- **Frequent Disconnections**: Verify firewall settings and WebSocket support
- **Authentication Errors**: Ensure valid session and proper token generation

### Trial Execution Problems

- **Steps Not Loading**: Verify experiment has published steps in database
- **Robot Commands Failing**: Check robot connection and plugin configuration
- **Progress Not Updating**: Confirm WebSocket messages are being sent/received

### Recovery Procedures

1. **Connection Loss**: Interface automatically attempts reconnection with exponential backoff
2. **Trial State Mismatch**: Use "Refresh" button to sync with server state
3. **Robot Disconnect**: Monitor robot status panel for connection recovery

## Best Practices

### Wizard Guidelines

1. **Pre-Trial Preparation**
   - Review complete experiment protocol
   - Test robot functionality before participant arrival
   - Verify audio/video recording systems

2. **During Trial Execution**
   - Follow protocol steps in sequence
   - Use intervention logging for any deviations
   - Monitor participant comfort and engagement
   - Watch robot status for any issues

3. **Post-Trial Procedures**
   - Complete trial properly (don't just abort)
   - Add summary notes about participant behavior
   - Review event log for any anomalies

### Technical Considerations

- **Browser Compatibility**: Use modern browsers with WebSocket support
- **Network Requirements**: Stable internet connection for real-time features
- **Performance**: Close unnecessary browser tabs during trials
- **Backup Plans**: Have manual procedures ready if technology fails

## Development and Customization

### Adding Custom Actions

```typescript
// Register new wizard action
const handleCustomAction = async (actionData: Record<string, unknown>) => {
  await logEventMutation.mutateAsync({
    trialId: trial.id,
    type: "wizard_action",
    data: {
      action_type: "custom_intervention",
      ...actionData
    }
  });
};
```

### Extending Robot Support

1. Create new robot plugin following plugin system guidelines
2. Define action schemas in plugin configuration
3. Implement communication protocol (REST/ROS2/WebSocket)
4. Test integration with wizard interface

### Custom Step Types

To add new step types:
1. Update database schema (`stepTypeEnum`)
2. Add type mapping in `WizardInterface.tsx`
3. Create step-specific UI components
4. Update execution engine logic

## Security Considerations

- **Authentication**: All WebSocket connections require valid session tokens
- **Authorization**: Role-based access control for trial operations
- **Data Protection**: All trial data encrypted in transit and at rest
- **Session Management**: Automatic cleanup of expired connections

## Performance Optimization

- **Connection Pooling**: Efficient WebSocket connection management
- **Event Batching**: Group related events to reduce message overhead
- **Selective Updates**: Only broadcast relevant changes to connected clients
- **Caching**: Local state management for responsive UI updates

---

## Quick Start Checklist

- [ ] Database seeded with test data (`bun db:seed`)
- [ ] Development server running (`bun dev`)
- [ ] Logged in as administrator (sean@soconnor.dev)
- [ ] Navigate to Trials section
- [ ] Select a trial and click "Wizard Control"
- [ ] Verify WebSocket connection (green "Real-time" badge)
- [ ] Start trial and execute steps
- [ ] Monitor robot status and event log
- [ ] Complete trial and review analysis page

For additional support, refer to the complete HRIStudio documentation in the `docs/` folder.