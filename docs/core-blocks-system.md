# HRIStudio Core Blocks System

## Overview

The core blocks system provides essential building blocks for the visual experiment designer through a repository-based plugin architecture. This system ensures consistency, extensibility, and maintainability by treating all blocks (core functionality and robot actions) as plugins loaded from repositories.

**Quick Links:**
- [Plugin System Implementation Guide](plugin-system-implementation-guide.md)
- [Work in Progress](work_in_progress.md#core-block-system-implementation-february-2024)
- [Project Overview](project-overview.md#2-visual-experiment-designer-ede)

## ✅ **Implementation Complete**

### **1. Core Repository Structure**

Created `hristudio-core/` repository with complete plugin architecture:

```
hristudio-core/
├── repository.json           # Repository metadata
├── plugins/
│   ├── index.json           # Plugin index (26 total blocks)
│   ├── events.json          # Event trigger blocks (4 blocks)
│   ├── wizard-actions.json  # Wizard action blocks (6 blocks)
│   ├── control-flow.json    # Control flow blocks (8 blocks)
│   └── observation.json     # Observation blocks (8 blocks)
├── assets/                  # Repository assets
└── README.md               # Complete documentation
```

### **2. Block Categories Implemented**

#### **Event Triggers (4 blocks)**
- `when_trial_starts` - Trial initialization trigger
- `when_participant_speaks` - Speech detection with duration threshold
- `when_timer_expires` - Time-based triggers with custom delays
- `when_key_pressed` - Wizard keyboard shortcuts

#### **Wizard Actions (6 blocks)**
- `wizard_say` - Speech with tone guidance
- `wizard_gesture` - Physical gestures with directions
- `wizard_show_object` - Object presentation with action types
- `wizard_record_note` - Observation recording with categorization
- `wizard_wait_for_response` - Response waiting with timeout
- `wizard_rate_interaction` - Subjective rating scales

#### **Control Flow (8 blocks)**
- `wait` - Pause execution with optional countdown
- `repeat` - Loop execution with delay between iterations
- `if_condition` - Conditional logic with multiple condition types
- `parallel` - Simultaneous execution with timeout controls
- `sequence` - Sequential execution with error handling
- `random_choice` - Weighted random path selection
- `try_catch` - Error handling with retry mechanisms
- `break` - Exit controls for loops/sequences/trials

#### **Observation & Sensing (8 blocks)**
- `observe_behavior` - Behavioral coding with standardized scales
- `measure_response_time` - Stimulus-response timing measurement
- `count_events` - Event frequency tracking
- `record_audio` - Audio capture with quality settings
- `capture_video` - Multi-camera video recording
- `log_event` - Timestamped event logging
- `survey_question` - In-trial questionnaires
- `physiological_measure` - Sensor data collection

### **3. Technical Architecture Changes**

#### **BlockRegistry Refactoring**
- **Removed**: All hardcoded core blocks (`initializeCoreBlocks()`)
- **Added**: Async `loadCoreBlocks()` method with repository fetching
- **Improved**: Error handling, fallback system, type safety
- **Enhanced**: Logging and debugging capabilities

#### **Dynamic Loading System**
```typescript
async loadCoreBlocks() {
  // Fetch blocks from /hristudio-core/plugins/
  // Parse and validate JSON structures
  // Convert to PluginBlockDefinition format
  // Register with BlockRegistry
  // Fallback to minimal blocks if loading fails
}
```

#### **Public Serving**
- Core repository copied to `public/hristudio-core/`
- Accessible via `/hristudio-core/plugins/*.json`
- Static serving ensures reliable access

### **4. Files Created/Modified**

#### **New Files**
- `hristudio-core/repository.json` - Repository metadata
- `hristudio-core/plugins/events.json` - Event blocks (4)
- `hristudio-core/plugins/wizard-actions.json` - Wizard blocks (6)
- `hristudio-core/plugins/control-flow.json` - Control blocks (8)
- `hristudio-core/plugins/observation.json` - Observation blocks (8)
- `hristudio-core/plugins/index.json` - Plugin index
- `hristudio-core/README.md` - Complete documentation
- `public/hristudio-core/` - Public serving copy
- `scripts/test-core-blocks.ts` - Validation test script

#### **Modified Files**
- `src/components/experiments/designer/EnhancedBlockDesigner.tsx`
  - Replaced hardcoded blocks with dynamic loading
  - Enhanced error handling and type safety
  - Improved plugin loading integration
- `scripts/seed-core-blocks.ts` - Fixed imports and type errors
- `scripts/seed-plugins.ts` - Fixed operators and imports  
- `scripts/seed.ts` - Fixed delete operations warnings

### **5. Quality Assurance**

#### **Validation System**
- JSON schema validation for all block definitions
- Type consistency checking (category colors, required fields)
- Parameter validation (types, constraints, options)
- Comprehensive test coverage

#### **Test Results**
```
✅ All tests passed! Core blocks system is working correctly.
   • 26 blocks loaded from repository
   • All required core blocks present
   • Registry loading simulation successful
```

#### **TypeScript Compliance**
- Fixed all unsafe `any` type usage
- Proper type definitions for all block structures
- Nullish coalescing operators throughout
- No compilation errors or warnings

### **6. Benefits Achieved**

#### **Complete Consistency**
- All blocks (core + robot) now use identical plugin architecture
- Unified block management and loading patterns
- Consistent JSON schema and validation

#### **Enhanced Extensibility**
- Add new core blocks by editing JSON files (no code changes)
- Version control for core functionality
- Independent updates and rollbacks

#### **Improved Maintainability**
- Centralized block definitions
- Clear separation of concerns
- Comprehensive documentation and validation

#### **Better Developer Experience**
- Type-safe block loading
- Detailed error messages and logging
- Fallback system ensures robustness

### **7. Integration Points**

#### **Experiment Designer**
- Automatic core blocks loading on component mount
- Seamless integration with existing plugin system
- Consistent block palette organization

#### **Database Integration**
- Core blocks can be seeded as plugins if needed
- Compatible with existing plugin management system
- Study-scoped installations possible

#### **Future Extensibility**
- Easy to create additional core repositories
- Simple to add new block categories
- Version management ready

## **Technical Specifications**

### **Block Definition Schema**
```json
{
  "id": "block_identifier",
  "name": "Display Name",
  "description": "Block description",
  "category": "event|wizard|control|sensor",
  "shape": "hat|action|control|boolean|value",
  "icon": "LucideIconName",
  "color": "#hexcolor",
  "parameters": [...],
  "execution": {...}
}
```

### **Loading Process**
1. Component mount triggers `loadCoreBlocks()`
2. Fetch each block set from `/hristudio-core/plugins/`
3. Validate JSON structure and block definitions
4. Convert to `PluginBlockDefinition` format
5. Register with `BlockRegistry`
6. Fallback to minimal blocks if any failures

### **Error Handling**
- Network failures → fallback blocks
- Invalid JSON → skip block set with warning
- Invalid blocks → skip individual blocks
- Type errors → graceful degradation

## Integration with HRIStudio Platform

### Related Documentation
- **[Plugin System Implementation Guide](plugin-system-implementation-guide.md)** - Robot plugin architecture
- **[Implementation Details](implementation-details.md)** - Overall platform architecture
- **[Database Schema](database-schema.md)** - Plugin storage and management tables
- **[API Routes](api-routes.md)** - Plugin management endpoints

### Development Commands
```bash
# Test core blocks loading
bun run scripts/test-core-blocks.ts

# Validate block definitions
cd public/hristudio-core && node validate.js

# Update public repository
cp -r hristudio-core/ public/hristudio-core/
```

## Status: Production Ready

✅ **Complete**: All 26 core blocks implemented and tested  
✅ **Validated**: Comprehensive testing and type checking  
✅ **Documented**: Integrated with existing documentation system  
✅ **Integrated**: Seamless experiment designer integration  
✅ **Extensible**: Ready for future enhancements

The core blocks system provides a robust, maintainable foundation for HRIStudio's experiment designer, ensuring complete architectural consistency across all platform components.