# HRIStudio Thesis Implementation - Fall 2025

**Sean O'Connor - CS Honors Thesis**  
**Advisor**: L. Felipe Perrone  
**Defense**: April 2026

## Implementation Status

Core platform infrastructure exists but MVP requires wizard interface implementation and robot control integration for functional trials.

## Fall Development Sprint (10-12 weeks)

| Sprint | Focus Area | Key Tasks | Success Metric |
|--------|------------|-----------|----------------|
| 1 (3 weeks) | Wizard Interface MVP | Trial control interface<br/>Step navigation<br/>Action execution buttons | Functional wizard interface for trial control |
| 2 (4 weeks) | Robot Integration | NAO6 API integration<br/>Basic action implementation<br/>Error handling and recovery | Wizard button → robot action |
| 3 (3 weeks) | Real-time Infrastructure | WebSocket server implementation<br/>Multi-client session management<br/>Event broadcasting system | Multiple users connected to live trial |
| 4 (2 weeks) | Integration Testing | Complete workflow validation<br/>Reliability testing<br/>Mock robot mode | 30-minute trials without crashes |

## User Study Preparation (4-5 weeks)

| Task Category | Deliverables | Effort |
|---------------|--------------|--------|
| Study Design | Reference experiment selection<br/>Equivalent implementations (HRIStudio + Choregraphe)<br/>Protocol validation | 3 weeks |
| Research Setup | IRB application submission<br/>Training material development<br/>Participant recruitment | 2 weeks |

## MVP Implementation Priorities

| Priority | Component | Current State | Target State |
|----------|-----------|---------------|-------------|
| **P0** | Wizard Interface | Design exists, needs implementation | Functional trial control interface |
| **P0** | Robot Control | Simulated responses only | Live NAO6 hardware control |
| **P0** | Real-time Communication | Client hooks exist, no server | Multi-user live trial coordination |
| **P1** | Trial Execution | Basic framework exists | Integrated with wizard + robot hardware |
| **P2** | Data Capture | Basic logging | Comprehensive real-time events |

## Success Criteria by Phase

### MVP Complete (10-12 weeks)
- [ ] Wizard interface allows trial control and step navigation
- [ ] Psychology researcher clicks interface → NAO6 performs action
- [ ] Multiple observers watch trial with live updates
- [ ] System remains stable during full experimental sessions
- [ ] All trial events captured with timestamps

### Study Ready (14-17 weeks)
- [ ] Reference experiment works identically in both platforms
- [ ] IRB approval obtained for comparative study
- [ ] 10-12 participants recruited from target disciplines
- [ ] Platform validated with non-technical users

## MVP Backlog - Priority Breakdown

### P0 - Critical MVP Features
| Story | Effort | Definition of Done |
|-------|--------|-------------------|
| Wizard interface trial control | 2 weeks | Interface for starting/stopping trials, navigating steps |
| Action execution buttons | 1 week | Buttons for robot actions with real-time feedback |
| NAO6 API integration | 3 weeks | Successfully connect to NAO6, execute basic commands |
| Basic robot actions | 2 weeks | Speech, movement, posture actions working |
| WebSocket server implementation | 2 weeks | Server accepts connections, handles authentication |
| Multi-client session management | 1 week | Multiple users can join same trial session |

### P1 - High Priority Features  
| Story | Effort | Definition of Done |
|-------|--------|-------------------|
| Event broadcasting system | 1 week | Actions broadcast to all connected clients |
| Robot status monitoring | 1 week | Connection status, error detection |
| End-to-end workflow testing | 1.5 weeks | Complete trial execution with real robot |

### P2 - Backlog (Post-MVP)
| Story | Effort | Definition of Done |
|-------|--------|-------------------|
| Connection recovery mechanisms | 1 week | Auto-reconnect on disconnect, graceful fallback |
| Mock robot development mode | 0.5 weeks | Development without hardware dependency |
| Performance optimization | 0.5 weeks | Response times under acceptable thresholds |
| Advanced data capture | 1 week | Comprehensive real-time event logging |

## User Study Framework

**Participants**: 10-12 researchers from Psychology/Education  
**Task**: Recreate published HRI experiment  
**Comparison**: HRIStudio (experimental) vs Choregraphe (control)  
**Measures**: Protocol accuracy, completion time, user experience ratings

## Implementation Strategy

Core platform infrastructure exists but wizard interface needs full implementation alongside robot integration. Focus on MVP that enables basic trial execution with real robot control.

**Critical Path**: Wizard interface → WebSocket server → NAO6 integration → end-to-end testing → user study execution