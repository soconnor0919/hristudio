# HRIStudio Documentation

HRIStudio is a web-based Wizard-of-Oz platform for Human-Robot Interaction research.

## Quick Links

| Document | Description |
|----------|-------------|
| **[Quick Reference](quick-reference.md)** | Essential commands, setup, troubleshooting |
| **[Project Status](project-status.md)** | Current development state (March 2026) |
| **[Implementation Guide](implementation-guide.md)** | Full technical implementation |
| **[NAO6 Integration](nao6-quick-reference.md)** | Robot setup and commands |

## Getting Started

### 1. Clone & Install
```bash
git clone https://github.com/soconnor0919/hristudio.git
cd hristudio
git submodule update --init --recursive
bun install
```

### 2. Start Database
```bash
bun run docker:up
bun db:push
bun db:seed
```

### 3. Start Application
```bash
bun dev
# Visit http://localhost:3000
# Login: sean@soconnor.dev / password123
```

### 4. Start NAO6 Robot (optional)
```bash
cd ~/Documents/Projects/nao6-hristudio-integration
docker compose up -d
```

## Current Architecture

```
┌──────────────────────────────────────────────────────────┐
│                    HRIStudio Platform                    │
├──────────────────────────────────────────────────────────┤
│ UI Layer (Next.js + React + shadcn/ui)                  │
│   ├── Experiment Designer (drag-and-drop)               │
│   ├── Wizard Interface (real-time trial execution)      │
│   └── Observer/Participant Views                        │
├──────────────────────────────────────────────────────────┤
│ Logic Layer (tRPC + Better Auth)                      │
│   ├── 12 tRPC routers (studies, experiments, trials...)  │
│   ├── Role-based authentication (4 roles)                │
│   └── WebSocket for real-time updates                   │
├──────────────────────────────────────────────────────────┤
│ Data Layer (PostgreSQL + Drizzle ORM)                   │
│   ├── 31 tables with complete relationships             │
│   ├── Plugin system with identifier-based lookup        │
│   └── Comprehensive event logging                       │
├──────────────────────────────────────────────────────────┤
│ Robot Integration (ROS2 via WebSocket)                   │
│   Docker: nao_driver, ros_bridge, ros_api              │
│   Plugin identifier: "nao6-ros2"                         │
└──────────────────────────────────────────────────────────┘
```

## Key Features

- **Hierarchical Structure**: Study → Experiment → Trial → Step → Action
- **Visual Designer**: 26+ core blocks (events, wizard actions, control flow, observation)
- **Conditional Branching**: Wizard choices with convergence paths
- **WebSocket Real-time**: Trial updates with auto-reconnect
- **Plugin System**: Robot-agnostic via identifier lookup
- **Docker NAO6**: Three-service ROS2 integration
- **Forms System**: Consent forms, surveys, questionnaires with templates
- **Role-based Access**: Owner, Researcher, Wizard, Observer permissions

## System Components

### Backend (src/server/)
- `api/routers/` - 13 tRPC routers (studies, experiments, trials, participants, forms, etc.)
- `db/schema.ts` - Drizzle schema (33 tables)
- `services/trial-execution.ts` - Trial execution engine
- `services/websocket-manager.ts` - Real-time connections

### Frontend (src/)
- `app/` - Next.js App Router pages
- `components/trials/wizard/` - Wizard interface
- `components/trials/forms/` - Form builder and viewer
- `hooks/useWebSocket.ts` - Real-time trial updates
- `lib/ros/wizard-ros-service.ts` - Robot control

## Plugin Identifier System

```typescript
// Plugins table has:
// - identifier: "nao6-ros2" (unique, machine-readable)
// - name: "NAO6 Robot (ROS2 Integration)" (display)

// Lookup order in trial execution:
1. Look up by identifier (e.g., "nao6-ros2")
2. Fall back to name (e.g., "NAO6 Robot")
3. Return null if not found
```

## Branching Flow

```
Step 3 (Comprehension Check)
  └── wizard_wait_for_response
      ├── "Correct" → nextStepId = step4a.id
      └── "Incorrect" → nextStepId = step4b.id

Step 4a/4b (Branch A/B)
  └── conditions.nextStepId: step5.id → converge

Step 5 (Story Continues)
  └── Linear progression to conclusion
```

## Development Workflow

```bash
# Make changes
# ...

# Validate
bun typecheck
bun lint

# Push schema (if changed)
bun db:push

# Reseed (if data changed)
bun db:seed
```

## Common Issues

| Issue | Solution |
|-------|----------|
| Build errors | `rm -rf .next && bun build` |
| DB issues | `bun db:push --force && bun db:seed` |
| Type errors | Check `bun typecheck` output |
| WebSocket fails | Verify port 3001 available |

## External Resources

- [Thesis (honors-thesis)](https://github.com/soconnor0919/honors-thesis)
- [NAO6 Integration](https://github.com/soconnor0919/nao6-hristudio-integration)
- [Robot Plugins](https://github.com/soconnor0919/robot-plugins)

## File Index

### Primary Documentation
- `README.md` - Project overview
- `docs/README.md` - This file
- `docs/quick-reference.md` - Commands & setup
- `docs/nao6-quick-reference.md` - NAO6 commands

### Technical Documentation
- `docs/implementation-guide.md` - Full technical implementation
- `docs/project-status.md` - Development status

### Archive (Historical)
- `docs/_archive/` - Old documentation (outdated but preserved)

---

**Last Updated**: March 22, 2026