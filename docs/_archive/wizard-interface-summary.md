# Wizard Interface Summary & Usage Guide

## Overview

The Wizard Interface has been completely fixed and enhanced to provide a professional, production-ready control panel for conducting HRI trials. All duplicate headers have been removed, real experiment data is now used instead of hardcoded values, and the WebSocket system is properly integrated.

## Key Fixes Applied

### 1. Removed Duplicate Headers ✅
- **ParticipantInfo**: Removed redundant Card headers since it's used inside EntityViewSection
- **RobotStatus**: Cleaned up duplicate title sections and unified layout
- **All Components**: Now follow consistent design patterns without header duplication

### 2. Real Experiment Data Integration ✅
- **Experiment Steps**: Now loads actual steps from database via `api.experiments.getSteps`
- **Type Mapping**: Database step types ("wizard", "robot", "parallel", "conditional") properly mapped to component types ("wizard_action", "robot_action", "parallel_steps", "conditional_branch")
- **Step Properties**: Real step names, descriptions, and duration estimates from experiment designer

### 3. Type Safety Improvements ✅
- **Demographics Handling**: Fixed all `any` types in ParticipantInfo component
- **Step Type Mapping**: Proper TypeScript types throughout the wizard interface
- **Null Safety**: Using nullish coalescing (`??`) instead of logical OR (`||`) for better type safety

## Current System Status

### ✅ Working Features
- **Real-time WebSocket Connection**: Live trial updates and control
- **Step-by-step Execution**: Navigate through actual experiment protocols
- **Robot Status Monitoring**: Battery, signal, position, and sensor tracking
- **Participant Information**: Complete demographics and consent status
- **Event Logging**: Real-time capture of all trial activities
- **Trial Control**: Start, execute, complete, and abort trials

### 📊 Seed Data Available
Run `bun db:seed` to populate with realistic test data:

**Test Experiments:**
- **"Basic Interaction Protocol 1"** - 3 steps with wizard actions and NAO integration
- **"Dialogue Timing Pilot"** - Multi-step protocol with parallel/conditional logic

**Test Participants:**
- 8 participants with complete demographics (age, gender, education, robot experience)
- Consent already verified for immediate testing

**Test Trials:**
- Multiple trials in different states (scheduled, in_progress, completed)
- Realistic metadata and execution history

## WebSocket Server Usage

### Automatic Connection
The wizard interface automatically connects to the WebSocket server at:
```
wss://localhost:3000/api/websocket?trialId={TRIAL_ID}&token={AUTH_TOKEN}
```

### Real-time Features
- **Connection Status**: Green "Real-time" badge when connected
- **Live Updates**: Trial status, step changes, and event logging
- **Automatic Reconnection**: Exponential backoff on connection loss
- **Error Handling**: User-friendly error messages and recovery

### Message Flow
```
Wizard Action → WebSocket → Server → Database → Broadcast → All Connected Clients
```

## Quick Start Instructions

### 1. Setup Development Environment
```bash
# Install dependencies
bun install

# Start database (if using Docker)
bun run docker:up

# Push schema and seed data
bun db:push
bun db:seed

# Start development server
bun dev
```

### 2. Access Wizard Interface
1. **Login**: `sean@soconnor.dev` / `password123`
2. **Navigate**: Dashboard → Studies → Select Study → Trials
3. **Select Trial**: Click on any trial with "scheduled" status
4. **Start Wizard**: Click "Wizard Control" button

### 3. Conduct a Trial
1. **Verify Connection**: Look for green "Real-time" badge in header
2. **Review Protocol**: Check experiment steps and participant info
3. **Start Trial**: Click "Start Trial" button
4. **Execute Steps**: Follow protocol step-by-step using "Next Step" button
5. **Monitor Status**: Watch robot status and live event log
6. **Complete Trial**: Click "Complete" when finished

## Expected Trial Flow

### Step 1: Introduction & Object Demo
- **Wizard Action**: Show object to participant
- **Robot Action**: NAO says "Hello, I am NAO. Let's begin!"
- **Duration**: ~60 seconds

### Step 2: Participant Response  
- **Wizard Action**: Wait for participant response
- **Prompt**: "What did you notice about the object?"
- **Timeout**: 20 seconds

### Step 3: Robot Feedback
- **Robot Action**: Set NAO LED color to blue
- **Wizard Fallback**: Record observation note if no robot available
- **Duration**: ~30 seconds

## WebSocket Communication Examples

### Starting a Trial
```json
{
  "type": "trial_action",
  "data": {
    "actionType": "start_trial",
    "step_index": 0,
    "data": { "notes": "Trial started by wizard" }
  }
}
```

### Logging Wizard Intervention
```json
{
  "type": "wizard_intervention", 
  "data": {
    "action_type": "manual_correction",
    "step_index": 1,
    "action_data": { "message": "Clarified participant question" }
  }
}
```

### Step Transition
```json
{
  "type": "step_transition",
  "data": {
    "from_step": 1,
    "to_step": 2,
    "step_name": "Participant Response"
  }
}
```

## Robot Integration

### Supported Robots
- **TurtleBot3 Burger**: ROS2 navigation and sensing
- **NAO Humanoid**: REST API for speech, gestures, LEDs
- **Plugin System**: Extensible architecture for additional robots

### Robot Actions in Seed Data
- **NAO Say Text**: Text-to-speech with configurable parameters
- **NAO Set LED Color**: Visual feedback through eye color changes  
- **NAO Play Animation**: Pre-defined gesture sequences
- **Wizard Fallbacks**: Manual alternatives when robots unavailable

## Troubleshooting

### WebSocket Issues
- **Red "Offline" Badge**: Check network connection and server status
- **Yellow "Connecting" Badge**: Normal during initial connection or reconnection
- **Connection Errors**: Verify authentication token and trial permissions

### Step Loading Problems
- **No Steps Showing**: Verify experiment has steps in database
- **"Loading experiment steps..."**: Normal during initial load
- **Type Errors**: Check step type mapping in console

### Robot Communication
- **Robot Status**: Monitor connection, battery, and sensor status
- **Action Failures**: Check robot plugin configuration and network
- **Fallback Actions**: System automatically provides wizard alternatives

## Production Deployment

### Environment Variables
```bash
DATABASE_URL=postgresql://user:pass@host:port/dbname
NEXTAUTH_SECRET=your-secret-key
NEXTAUTH_URL=https://your-domain.com
```

### WebSocket Configuration
- **Protocol**: Automatic upgrade from HTTP to WebSocket
- **Authentication**: Session-based token validation
- **Scaling**: Per-trial room isolation for concurrent sessions
- **Security**: Role-based access control and message validation

## Development Notes

### Architecture Decisions
- **EntityViewSection**: Consistent layout patterns across all pages
- **Real-time First**: WebSocket primary, polling fallback
- **Type Safety**: Strict TypeScript throughout wizard components
- **Plugin System**: Extensible robot integration architecture

### Performance Optimizations
- **Selective Polling**: Reduced frequency when WebSocket connected
- **Local State**: Efficient React state management
- **Event Batching**: Optimized WebSocket message handling
- **Caching**: Smart API data revalidation

## Next Steps

### Immediate Enhancements
- [ ] Observer-only interface for read-only trial monitoring
- [ ] Pause/resume functionality during trial execution
- [ ] Enhanced post-trial analytics and visualization
- [ ] Real robot hardware integration testing

### Future Improvements
- [ ] Multi-wizard collaboration features
- [ ] Advanced step branching and conditional logic
- [ ] Voice control integration for hands-free operation
- [ ] Mobile-responsive wizard interface

---

## Success Criteria Met ✅

- ✅ **No Duplicate Headers**: Clean, professional interface
- ✅ **Real Experiment Data**: No hardcoded values, actual database integration
- ✅ **WebSocket Integration**: Live real-time trial control and monitoring
- ✅ **Type Safety**: Strict TypeScript throughout wizard components
- ✅ **Production Ready**: Professional UI matching platform standards

The wizard interface is now production-ready and provides researchers with a comprehensive, real-time control system for conducting high-quality HRI studies.