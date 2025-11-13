# Experiment Designer Step Integration (Modular Architecture + Drift Handling)

## Overview

The HRIStudio experiment designer has been redesigned with a step-based + provenance-aware architecture that provides intuitive experiment creation, transparent plugin usage, and reproducible execution through integrity hashing and a compiled execution graph.

## Architecture

### Core Design Philosophy

The designer follows a clear hierarchy that matches database structure, runtime execution, and reproducibility tracking:
- **Experiment** → **Steps** → **Actions** (with provenance + execution descriptors)
- Steps are primary containers in the flow (Step 1 → Step 2 → Step 3) with sortable ordering
- Actions are dragged from a categorized library into step containers (core vs plugin clearly labeled)
- Direct 1:1 mapping to database `steps` and `actions` tables, persisting provenance & transport metadata

### Key Components (Post-Modularization)

#### ActionRegistry (`ActionRegistry.ts`)
- Loads actions from core plugin repositories (`hristudio-core/plugins/`)
- Integrates study-scoped robot plugins (namespaced: `pluginId.actionId`)
- Provides fallback actions if plugin loading fails (ensures minimal operability)
- Maps plugin parameter schemas (primitive: text/number/select/boolean) to UI fields
- Retains provenance + execution descriptors (transport, timeout, retryable)

#### Step-Based Flow (`StepFlow.tsx`)
- Sortable step containers with drag-and-drop reordering (via `@dnd-kit`)
- Color-coded step types (sequential, parallel, conditional, loop) with left border accent
- Expandable/collapsible view for managing complex experiments
- Visual connectors between steps (light vertical separators)
- Isolated from parameter/editor logic for performance and clarity

#### Action Library (`ActionLibrary.tsx`)
- Categorized tabs: Wizard (blue), Robot (emerald), Control (amber), Observation (purple)
- Tooltips show description, parameters, provenance badge (C core / P plugin)
- Drag-and-drop from library directly into specific step droppable zones
- Footer statistics (total actions / category count)
- Empty + fallback guidance when plugin actions absent
- Guarantees availability: once the experiment's study context and its installed plugins are loaded, all corresponding plugin actions are registered and appear (guarded against duplicate loads / stale study mismatch)
- Plugin availability is study-scoped: only plugins installed for the experiment's parent study (via Plugin Store installation) are loaded and exposed; this ensures experiments cannot reference uninstalled or unauthorized plugin actions.

#### Properties Panel (`PropertiesPanel.tsx`)
- Context-aware: action selection → action parameters; step selection → step metadata; otherwise instructional state
- Boolean parameters now render as accessible Switch
- Number parameters with `min`/`max` render as Slider (shows live value + bounds)
- Number parameters without bounds fall back to numeric input
- Select/text unchanged; future: enum grouping + advanced editors
- Displays provenance + transport badges (plugin id@version, transport, retryable)

## User Experience

### Visual Design
- **Tightened Spacing**: Compact UI with efficient screen real estate usage
- **Dark Mode Support**: Proper selection states and theme-aware colors
- **Color Consistency**: Category colors used throughout for visual coherence
- **Micro-interactions**: Hover states, drag overlays, smooth transitions

### Interaction Patterns
- **Direct Action Editing**: Click any action to immediately edit properties (no step selection required)
- **Multi-level Sorting**: Reorder steps in flow, reorder actions within steps
- **Visual Feedback**: Drop zones highlight, selection states clear, drag handles intuitive
- **Touch-friendly**: Proper activation constraints for mobile/touch devices

### Properties Panel (Enhanced Parameter Controls)
- **Action-First Workflow**: Immediate property editing on action selection
- **Rich Metadata**: Icon, category color, provenance badges (Core/Plugin, transport)
- **Switch for Boolean**: Improves clarity vs checkbox in dense layouts
- **Slider for Ranged Number**: Applies when `min` or `max` present (live formatted value)
- **Graceful Fallbacks**: Plain number input if no bounds; text/select unchanged
- **Context-Aware**: Step editing (name/type/trigger) isolated from action editing

## Technical Implementation

### Drag and Drop System
Built with `@dnd-kit` for robust, accessible drag-and-drop:

```typescript
// Multi-context sorting support
const handleDragEnd = (event: DragEndEvent) => {
  // Action from library to step
  if (activeId.startsWith("action-") && overId.startsWith("step-")) {
    // Add action to step
  }
  
  // Step reordering in flow
  if (!activeId.startsWith("action-") && !overId.startsWith("step-")) {
    // Reorder steps
  }
  
  // Action reordering within step
  if (!activeId.startsWith("action-") && !overId.startsWith("step-")) {
    // Reorder actions in step
  }
};
```

### Plugin Integration (Registry-Centric)
Actions are loaded dynamically from multiple sources with provenance & version retention:

```typescript
class ActionRegistry {
  async loadCoreActions() {
    // Load from hristudio-core/plugins/
    const coreActionSets = ["wizard-actions", "control-flow", "observation"];
    // Process and register actions
  }
  
  loadPluginActions(studyId: string, studyPlugins: Array<{plugin: any}>) {
    // Load robot-specific actions from study plugins
    // Map parameter schemas to form controls
  }
}
```

### Database & Execution Conversion
Two-layer conversion:
1. Visual design → DB steps/actions with provenance & execution metadata
2. Visual design → Compiled execution graph (normalized actions + transport summary + integrity hash)

```typescript
function convertStepsToDatabase(steps: ExperimentStep[]): ConvertedStep[] {
  return steps.map((step, index) => ({
    name: step.name,
    type: mapStepTypeToDatabase(step.type),
    orderIndex: index,
    conditions: step.trigger.conditions,
    actions: step.actions.map((action, actionIndex) => ({
      name: action.name,
      type: action.type,
      orderIndex: actionIndex,
      parameters: action.parameters,
    })),
  }));
}
```

## Validation & Hash Drift Handling
A validation workflow now surfaces structural integrity + reproducibility signals:

### Validation Endpoint
- `experiments.validateDesign` returns:
  - `valid` + `issues[]`
  - `integrityHash` (deterministic structural hash from compiled execution graph)
  - `pluginDependencies` (sorted, namespaced with versions)
  - Execution summary (steps/actions/transport mix)

### Drift Detection (Client-Side)
- Local state caches: `lastValidatedHash` + serialized design snapshot
- Drift conditions:
  1. Stored experiment `integrityHash` ≠ last validated hash
  2. Design snapshot changed since last validation (structural or param changes)
- Badge States:
  - `Validated` (green outline): design unchanged since last validation and matches stored hash
  - `Drift` (destructive): mismatch or post-validation edits
  - `Unvalidated`: no validation performed yet

### Rationale
- Encourages explicit revalidation after structural edits
- Prevents silent divergence from compiled execution artifact
- Future: differentiate structural vs param-only drift (hash currently parameter-key-based)

### Planned Enhancements
- Hash stability tuning (exclude mutable free-text values if needed)
- Inline warnings on mutated steps/actions
- Optional auto-validate on save (configurable)

## Plugin System Integration

### Core Actions
Loaded from `hristudio-core/plugins/` repositories:
- **wizard-actions.json**: Wizard speech, gestures, instructions
- **control-flow.json**: Wait, conditional logic, loops
- **observation.json**: Behavioral coding, data collection, measurements

### Robot Actions
Dynamically loaded based on study configuration:
- Robot-specific actions from plugin repositories
- Parameter schemas automatically converted to form controls
- Platform-specific validation and constraints

### Fallback System
Essential actions available even if plugin loading fails:
- Basic wizard speech and gesture actions
- Core control flow (wait, conditional)
- Simple observation and data collection

## Example Usage

### Creating an Experiment
1. **Add Steps**: Click "Add Step" to create containers in the experiment flow
2. **Configure Steps**: Set name, type (sequential/parallel/conditional/loop), triggers
3. **Drag Actions**: Drag from categorized library into step drop zones
4. **Edit Properties**: Click actions to immediately edit parameters
5. **Reorder**: Drag steps in flow, drag actions within steps
6. **Save**: Direct conversion to database step/action records (provenance & execution metadata persisted)

### Visual Workflow
```
Action Library          Experiment Flow              Properties
┌─────────────┐        ┌──────────────────┐         ┌─────────────┐
│ [Wizard]    │        │ Step 1: Welcome  │         │ Action:     │
│ [Robot]     │ -----> │ ├ Wizard Says    │ ------> │ Wizard Says │
│ [Control]   │        │ ├ Wait 2s        │         │ Text: ...   │
│ [Observe]   │        │ └ Observe        │         │ Tone: ...   │
└─────────────┘        └──────────────────┘         └─────────────┘
```

## Benefits

### For Researchers
- **Intuitive Design**: Natural workflow matching experimental thinking
- **Visual Clarity**: Clear step-by-step experiment structure
- **Plugin Integration**: Access to full ecosystem of robot platforms
- **Direct Editing**: No complex nested selections required

### For System Architecture
- **Clean Separation**: Visual design vs execution logic clearly separated
- **Database Integrity**: Direct 1:1 mapping maintains relationships
- **Plugin Extensibility**: Easy integration of new robot platforms
- **Type Safety**: Complete TypeScript integration throughout

### For Development
- **Maintainable Code**: Clean component architecture with clear responsibilities
- **Performance**: Efficient rendering with proper React patterns
- **Error Handling**: Graceful degradation when plugins fail
- **Accessibility**: Built on accessible `@dnd-kit` foundation

## Modular Architecture Summary
| Module | Responsibility | Notes |
|--------|----------------|-------|
| `BlockDesigner.tsx` | Orchestration (state, save, validation, drift badges) | Thin controller after refactor |
| `ActionRegistry.ts` | Core + plugin action loading, provenance, fallback | Stateless across renders (singleton) |
| `ActionLibrary.tsx` | Categorized draggable palette | Performance-isolated |
| `StepFlow.tsx` | Sortable steps & actions, structural UI | No parameter logic |
| `PropertiesPanel.tsx` | Parameter + metadata editing (enhanced controls) | Switch + Slider integration |

## Future Enhancements

### Planned Features (Updated Roadmap)
- **Step Templates**: Reusable step patterns for common workflows
- **Visual Debugging**: Inline structural + provenance validation markers
- **Collaborative Editing**: Real-time multi-user design sessions
- **Advanced Conditionals**: Branching logic & guard editors (visual condition builder)
- **Structural Drift Granularity**: Distinguish param-value vs structural changes
- **Version Pin Diffing**: Detect plugin version upgrades vs design-pinned versions

### Integration Opportunities
- **Version Control**: Track experiment changes across iterations
- **A/B Testing**: Support for experimental variations within single design
- **Analytics Integration**: Step-level performance monitoring
- **Export Formats**: Convert to external workflow systems

This redesigned experiment designer now combines step-based structure, provenance tracking, transport-aware execution compilation, integrity hashing, and validation workflows to deliver reproducible, extensible, and transparent experimental protocols.