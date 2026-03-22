# HRIStudio Quick Reference Guide

## Quick Setup

```bash
# Clone with submodules
git clone https://github.com/soconnor0919/hristudio.git
cd hristudio
git submodule update --init --recursive

# Install and setup
bun install
bun run docker:up
bun db:push
bun db:seed

# Start
bun dev
```

**Login**: `sean@soconnor.dev` / `password123`

---

## Key Concepts

### Hierarchy
```
Study → Experiment → Trial → Step → Action
```

### User Roles (Study-level)
- **Owner**: Full study control, manage members
- **Researcher**: Design experiments, manage participants
- **Wizard**: Execute trials, control robot during sessions
- **Observer**: Read-only access to study data

### Plugin Identifier System
- `identifier`: Machine-readable key (e.g., `nao6-ros2`)
- `name`: Display name (e.g., `NAO6 Robot (ROS2 Integration)`)
- Lookup order: identifier → name → fallback

---

## Development Commands

| Command | Description |
|---------|-------------|
| `bun dev` | Start dev server |
| `bun build` | Production build |
| `bun typecheck` | TypeScript validation |
| `bun db:push` | Push schema changes |
| `bun db:seed` | Seed data + sync plugins + forms |
| `bun run docker:up` | Start PostgreSQL + MinIO |

## Forms System

### Form Types
- **Consent**: Legal/IRB consent documents with signature fields
- **Survey**: Multi-question questionnaires (ratings, multiple choice)
- **Questionnaire**: Custom data collection forms

### Templates (seeded by default)
- Informed Consent - Standard consent template
- Post-Session Survey - Participant feedback form
- Demographics - Basic demographic collection

### Routes
- `/studies/[id]/forms` - List forms
- `/studies/[id]/forms/new` - Create form (from template or scratch)
- `/studies/[id]/forms/[formId]` - View/edit form, preview, responses

---

## NAO6 Robot Docker

```bash
cd ~/Documents/Projects/nao6-hristudio-integration
docker compose up -d
```

**Services**: nao_driver, ros_bridge (:9090), ros_api

**Topics**:
- `/speech` - TTS
- `/cmd_vel` - Movement
- `/leds/eyes` - LEDs

---

## Architecture Layers

```
┌─────────────────────────────────────┐
│  UI: Design / Execute / Playback    │
├─────────────────────────────────────┤
│  Server: tRPC, Auth, Trial Logic    │
├─────────────────────────────────────┤
│  Data: PostgreSQL, File Storage     │
│  Robot: ROS2 via WebSocket         │
└─────────────────────────────────────┘
```

---

## WebSocket Architecture

- **Trial Updates**: `ws://localhost:3001/api/websocket`
- **ROS Bridge**: `ws://localhost:9090` (rosbridge)
- **Real-time**: Auto-reconnect with exponential backoff
- **Message Types**: trial_event, trial_status, connection_established

---

## Database Schema

### Core Tables
- `users` - Authentication
- `studies` - Research projects
- `experiments` - Protocol templates
- `trials` - Execution instances
- `steps` - Experiment phases
- `actions` - Atomic tasks
- `plugins` - Robot integrations (identifier column)
- `trial_events` - Execution logs

---

## Route Structure

```
/dashboard           - Global overview
/studies             - Study list
/studies/[id]        - Study details
/studies/[id]/experiments
/studies/[id]/trials
/studies/[id]/participants
/trials/[id]/wizard - Trial execution
/experiments/[id]/designer - Visual editor
```

---

## Troubleshooting

**Build errors**: `rm -rf .next && bun build`

**Database reset**: `bun db:push --force && bun db:seed`

**Check types**: `bun typecheck`

---

## Plugin System

```typescript
// Loading a plugin by identifier
const plugin = await trialExecution.loadPlugin("nao6-ros2");

// Action execution
await robot.execute("nao6-ros2.say_with_emotion", { text: "Hello" });
```

---

Last updated: March 2026