# HRIStudio - March 2026 Development Summary

## What We Did This Session

### 1. Docker Integration for NAO6 Robot
**Files**: `nao6-hristudio-integration/`

- Created `Dockerfile` with ROS2 Humble + naoqi packages
- Created `docker-compose.yaml` with 3 services: `nao_driver`, `ros_bridge`, `ros_api`
- Created `scripts/init_robot.sh` - Bash script to wake up robot via SSH when Docker starts
- Fixed autonomous life disable issue (previously used Python `naoqi` package which isn't on PyPI)

**Key insight**: Robot init via SSH + `qicli` calls instead of Python SDK

### 2. Plugin System Fixes
**Files**: `robot-plugins/plugins/nao6-ros2.json`, `src/lib/ros/wizard-ros-service.ts`

- **Topic fixes**: Removed `/naoqi_driver/` prefix from topics (driver already provides unprefixed topics)
- **say_with_emotion**: Fixed with proper NAOqi markup (`\rspd=120\^start(animations/...)`)
- **wave_goodbye**: Added animated speech with waving gesture
- **play_animation**: Added for predefined NAO animations
- **Sensor topics**: Fixed camera, IMU, bumper, sonar, touch topics (removed prefix)

### 3. Database Schema - Plugin Identifier
**Files**: `src/server/db/schema.ts`, `src/server/services/trial-execution.ts`

- Added `identifier` column to `plugins` table (unique, machine-readable ID like `nao6-ros2`)
- `name` now for display only ("NAO6 Robot (ROS2 Integration)")
- Updated trial-execution to look up by `identifier` first, then `name` (backwards compat)
- Created migration script: `scripts/migrate-add-identifier.ts`

### 4. Seed Script Improvements
**Files**: `scripts/seed-dev.ts`

- Fixed to use local plugin file (not remote `repo.hristudio.com`)
- Added `identifier` field for all plugins (nao6, hristudio-core, hristudio-woz)
- Experiment structure:
  - Step 1: The Hook
  - Step 2: The Narrative
  - Step 3: Comprehension Check (conditional with wizard choices)
  - Step 4a/4b: Branch A/B (with `nextStepId` conditions to converge)
  - Step 5: Story Continues (convergence point)
  - Step 6: Conclusion

### 5. Robot Action Timing Fix
**Files**: `src/lib/ros/wizard-ros-service.ts`

- Speech actions now estimate duration: `1500ms emotion overhead + word_count * 300ms`
- Added `say_with_emotion` and `wave_goodbye` as explicit built-in actions
- Fixed 100ms timeout that was completing actions before robot finished

### 6. Branching Logic Fixes (Critical!)
**Files**: `src/components/trials/wizard/`

**Bug 1**: `onClick={onNextStep}` passed event object instead of calling function
- Fixed: `onClick={() => onNextStep()}`

**Bug 2**: `onCompleted()` called after branch choice incremented action count
- Fixed: Removed `onCompleted()` call after branch selection

**Bug 3**: Branch A/B had no `nextStepId` condition, fell through to linear progression
- Fixed: Added `conditions.nextStepId: step5.id` to Branch A and B

**Bug 4**: Robot actions from previous step executed on new step (branching jumped but actions from prior step still triggered)
- Root cause: `completedActionsCount` not being reset properly
- Fixed: `handleNextStep()` now resets `completedActionsCount(0)` on explicit jump

### 7. Auth.js to Better Auth Migration (Attempted, Reverted)
**Status**: Incomplete - 41+ type errors remain

The migration requires significant changes to how `session.user.roles` is accessed since Better Auth doesn't include roles in session by default. Would need to fetch roles from database on each request.

**Recommendation**: Defer until more development time available.

---

## Current Architecture

### Plugin Identifier System
```
plugins table:
  - id: UUID (primary key)
  - identifier: varchar (unique, e.g. "nao6-ros2")
  - name: varchar (display, e.g. "NAO6 Robot (ROS2 Integration)")
  - robotId: UUID (optional FK to robots)
  - actionDefinitions: JSONB

actions table:
  - type: "plugin.action" (e.g., "nao6-ros2.say_with_emotion")
  - pluginId: varchar (references plugins.identifier)
```

### Branching Flow
```
Step 3 (Comprehension Check)
  └── wizard_wait_for_response action
      ├── Click "Correct" → setLastResponse("Correct") → nextStepId=step4a.id
      └── Click "Incorrect" → setLastResponse("Incorrect") → nextStepId=step4b.id

Step 4a/4b (Branches)
  └── conditions.nextStepId: step5.id → jump to Story Continues

Step 5 (Story Continues)
  └── Linear progression to Step 6

Step 6 (Conclusion)
  └── Trial complete
```

### ROS Topics (NAO6)
```
/speech          - Text-to-speech
/cmd_vel         - Velocity commands
/joint_angles    - Joint position commands
/camera/front/image_raw
/camera/bottom/image_raw
/imu/torso
/bumper
/{hand,head}_touch
/sonar/{left,right}
/info
```

---

## Known Issues / Remaining Work

1. **Auth.js to Better Auth Migration** - Deferred, requires significant refactoring
2. **robots.executeSystemAction** - Procedure not found error (fallback works but should investigate)
3. **say_with_emotion via WebSocket** - May need proper plugin config to avoid fallback

---

## Docker Deployment

```bash
cd nao6-hristudio-integration
docker compose up -d
```

Robot init runs automatically on startup (via `init_robot.sh`).

---

## Testing Checklist

- [x] Docker builds and starts
- [x] Robot wakes up (autonomous life disabled)
- [x] Seed script runs successfully
- [x] Trial executes with proper branching
- [x] Branch A → Story Continues (not Branch B)
- [x] Robot speaks with emotion (say_with_emotion)
- [x] Wave gesture works
- [ ] Robot movement (walk, turn) tested
- [ ] All NAO6 actions verified

---

*Last Updated: March 21, 2026*
