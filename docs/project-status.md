# HRIStudio Project Status

## Current Status: Active Development

**Project Version**: 1.0.0  
**Last Updated**: March 2026  
**Overall Completion**: 98%  
**Status**: Thesis research phase

---

## Executive Summary

HRIStudio is a complete platform for Wizard-of-Oz HRI research. Key milestones achieved:

### Recent Updates (March 2026)
- ✅ WebSocket real-time trial updates implemented
- ✅ Better Auth migration complete (replaced NextAuth.js)
- ✅ Docker integration for NAO6 (3 services: nao_driver, ros_bridge, ros_api)
- ✅ Conditional branching with wizard choices and convergence
- ✅ 14 NAO6 robot actions (speech, movement, gestures, sensors, LEDs, animations)
- ✅ Plugin identifier system for clean plugin lookup
- ✅ Seed script with branching experiment structure

### Key Achievements
- ✅ Complete backend with 12 tRPC routers
- ✅ Professional UI with unified experiences
- ✅ Full TypeScript coverage (strict mode)
- ✅ Role-based access control (4 roles)
- ✅ 31 database tables with relationships
- ✅ Experiment designer with 26+ core blocks
- ✅ Real-time trial execution wizard interface
- ✅ NAO6 robot integration via ROS2 Humble

---

## Architecture

### Three-Layer Architecture

```
┌─────────────────────────────────────────────────────┐
│  User Interface Layer                                │
│  ├── Experiment Designer (visual programming)        │
│  ├── Wizard Interface (trial execution)              │
│  ├── Observer View (live monitoring)                 │
│  └── Participant View (thesis study)                │
├─────────────────────────────────────────────────────┤
│  Data Management Layer                               │
│  ├── PostgreSQL + Drizzle ORM                        │
│  ├── tRPC API (12 routers)                           │
│  └── Better Auth (role-based auth)                │
├─────────────────────────────────────────────────────┤
│  Robot Integration Layer                             │
│  ├── Plugin system (robot-agnostic)                 │
│  ├── ROS2 via rosbridge WebSocket                   │
│  └── Docker deployment (nao_driver, ros_bridge)      │
└─────────────────────────────────────────────────────┘
```

### Plugin Identifier System

```
plugins table:
  - id: UUID (primary key)
  - identifier: varchar (unique, e.g. "nao6-ros2")
  - name: varchar (display, e.g. "NAO6 Robot (ROS2)")
  - robotId: UUID (optional FK to robots)
  - actionDefinitions: JSONB

actions table:
  - type: "plugin.action" (e.g., "nao6-ros2.say_with_emotion")
  - pluginId: varchar (references plugins.identifier)
```

---

## Branching Flow

Experiment steps support conditional branching with wizard choices:

```
Step 3 (Comprehension Check)
  └── wizard_wait_for_response
      ├── Click "Correct" → setLastResponse("Correct") → nextStepId=step4a
      └── Click "Incorrect" → setLastResponse("Incorrect") → nextStepId=step4b

Step 4a/4b (Branches)
  └── conditions.nextStepId: step5.id → convergence point

Step 5 (Story Continues)
  └── Linear progression to Step 6
```

---

## NAO6 Robot Actions (14 total)

| Category | Actions |
|----------|---------|
| Speech | say, say_with_emotion, wave_goodbye |
| Movement | walk, turn, move_to_posture |
| Gestures | play_animation, gesture |
| Sensors | get_sensors, bumper_state, touch_state |
| LEDs | set_eye_leds, set_breathing_lights |

---

## Tech Stack

| Component | Technology | Version |
|-----------|------------|---------|
| Framework | Next.js | 15-16.x |
| Language | TypeScript | 5.x (strict) |
| Database | PostgreSQL | 14+ |
| ORM | Drizzle | latest |
| Auth | NextAuth.js | v5 |
| API | tRPC | latest |
| UI | Tailwind + shadcn/ui | latest |
| Real-time | WebSocket | with polling fallback |
| Robot | ROS2 Humble | via rosbridge |
| Package Manager | Bun | latest |

---

## Development Status

### Completed Features
| Feature | Status | Notes |
|---------|--------|-------|
| Database Schema | ✅ | 31 tables |
| Authentication | ✅ | 4 roles |
| Experiment Designer | ✅ | 26+ blocks |
| Wizard Interface | ✅ | 3-panel design |
| Real-time Updates | ✅ | WebSocket |
| Plugin System | ✅ | Robot-agnostic |
| NAO6 Integration | ✅ | Docker deployment |
| Conditional Branching | ✅ | Wizard choices |
| Mock Robot | ✅ | Development mode |

### Known Issues
| Issue | Status | Notes |
|-------|--------|-------|
| robots.executeSystemAction | Known error | Fallback works |

---

## SSH Deployment Commands

```bash
# Local development
bun dev

# Database
bun db:push          # Push schema changes
bun db:seed          # Seed with test data
bun run docker:up   # Start PostgreSQL

# Quality
bun typecheck       # TypeScript validation
bun lint            # ESLint
```

---

## Thesis Timeline

Current phase: **March 2026** - Implementation complete, preparing user study

| Phase | Status | Date |
|-------|--------|------|
| Proposal | ✅ | Sept 2025 |
| IRB Application | ✅ | Dec 2025 |
| Implementation | ✅ | Feb 2026 |
| User Study | 🔄 In Progress | Mar-Apr 2026 |
| Defense | Scheduled | April 2026 |

---

## Next Steps

1. Complete user study (10-12 participants)
2. Data analysis and thesis writing
3. Final defense April 2026
4. Open source release

---

*Last Updated: March 22, 2026*