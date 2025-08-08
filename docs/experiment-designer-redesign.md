# Experiment Designer Redesign (Production Baseline)

This document defines the production-ready redesign of the Experiment Designer. It supersedes prior "modular refactor" notes and consolidates architecture, hashing, drift detection, UI/UX, state management, validation, plugin integration, saving/versioning, and export strategy. All implementation must adhere to this specification to ensure reproducibility, extensibility, and maintainability.

---

## 1. Goals

- Provide a clear, fast, and intuitive hierarchical design workflow.
- Guarantee reproducibility via deterministic hashing and provenance retention.
- Surface plugin dependency health and schema drift early.
- Support scalable experiments (many steps/actions) without UI degradation.
- Enable structured validation (structural, parameter-level, semantic) before compilation.
- Provide robust export/import and future multi-user collaboration readiness.
- Maintain 100% type safety and consistent design patterns across the platform.

---

## 2. Conceptual Model

Hierarchy:
```
Experiment
  └─ Step (ordered, typed)
        └─ Action (ordered, typed, provenance-bound)
```

Key invariants:
- Step `order` is zero-based, contiguous after any mutation.
- Action `orderIndex` is zero-based within its parent step.
- Each action retains provenance: `{ source.kind, pluginId?, pluginVersion?, baseActionId? }`
- Execution descriptors are retained for reproducibility (`transport`, `ros2`, `rest`, `retryable`, `timeoutMs`).

---

## 3. Hashing & Integrity Guarantees

### 3.1 Purposes of Hashing
- Detect structural drift between persisted design and working edits.
- Bind design to plugin and action schemas for reproducibility.
- Support execution graph provenance (hash at compile time).
- Enable export integrity and offline verification.

### 3.2 Design Hash Components
Included:
- Steps (ordered): `id`, `name`, `type`, `order`, `trigger.type`, sorted trigger condition keys.
- Actions (per step, ordered): `id`, `type`, `source.kind`, `pluginId`, `pluginVersion`, `baseActionId`, `execution.transport`, parameter key set (NOT values by default—configurable).
- Optional: parameter values (toggle design mode if needed—default: exclude values to reduce false-positive drift).

Excluded:
- Ephemeral UI state (`expanded`, selection state).
- Human-friendly timestamps or transient meta.

### 3.3 Canonicalization Steps
1. Deep clone design subset.
2. Remove `undefined` keys.
3. Sort object keys recursively.
4. Sort arrays where semantic ordering is not meaningful (e.g. condition key sets).
5. JSON stringify (no whitespace).
6. SHA-256 digest → hex.

### 3.4 Incremental Hash Optimization
Per-action hash → per-step hash → design hash:
```
H_action = hash(canonical(actionMeta))
H_step = hash(stepMeta + concat(H_action_i))
H_design = hash(globalMeta + concat(H_step_i))
```
Recompute only modified branches for responsiveness.

### 3.5 Drift States
| State        | Condition                                                                                          |
|--------------|----------------------------------------------------------------------------------------------------|
| Unvalidated  | No validation performed since load                                                                 |
| Validated    | Last validated hash matches stored experiment integrity hash and working snapshot unchanged        |
| Drift        | (A) Working snapshot changed since last validation OR (B) validated hash ≠ stored integrity hash   |
| Plugin Drift | Design validated, but one or more action definitions (schema/provenance) no longer match registry  |

### 3.6 Plugin Signature Drift
Signature hash per action definition: hash of `(type + category + parameterSchema + transport + baseActionId + pluginVersion)`.
- If signature mismatch for existing action instance: mark as “schema drift”.
- Provide reconciliation CTA to refetch schema and optionally run migration.

---

## 4. UI Architecture

High-level layout:

```
┌───────────────────────────────────────────────────────────────────────────┐
│ Header: breadcrumbs • experiment name • hash badge • plugin deps summary  │
├──────────────┬───────────────────────────────┬────────────────────────────┤
│ Action       │ Step Flow (sortable linear)   │ Properties / Inspector      │
│ Library      │ - Step cards (collapsible)     │ - Step editor               │
│ - Search     │ - Inline action list           │ - Action parameters         │
│ - Categories │ - DnD (steps/actions/library)  │ - Validation & drift panel  │
│ - Plugins    │ - Structural markers           │ - Dependencies & provenance │
├──────────────┴───────────────────────────────┴────────────────────────────┤
│ Bottom Save Bar: dirty state • versioning • export • last saved • conflicts│
└───────────────────────────────────────────────────────────────────────────┘
```

Component responsibilities:
| Component                     | Responsibility |
|------------------------------|----------------|
| `DesignerShell`              | Data loading, permission guard, store boot |
| `ActionLibraryPanel`         | Search/filter, categorized draggable items |
| `StepFlow`                   | Rendering + reordering steps & actions |
| `StepCard`                   | Step context container |
| `ActionItem`                 | Visual + selectable action row |
| `PropertiesPanel`            | Context editing (step/action) |
| `ParameterFieldFactory`      | Schema → control mapping |
| `ValidationPanel`            | Issue listing + filtering |
| `DependencyInspector`        | Plugin + action provenance health |
| `SaveBar`                    | Hash/drift/dirtiness/export/version controls |
| `hashing.ts`                 | Canonicalization + incremental hashing |
| `validators.ts`              | Rule execution (structural, parameter) |
| `exporters.ts`               | Export bundle builder |
| `store/useDesignerStore.ts`  | Central state store |

---

## 5. State Management

Use a lightweight, framework-agnostic, fully typed store (Zustand). Core fields:

```
{
  steps: ExperimentStep[]
  dirty: Set<string>
  selectedStepId?: string
  selectedActionId?: string
  lastPersistedHash?: string
  lastValidatedHash?: string
  lastValidatedSnapshot?: string
  pluginSignatureIndex: Record<actionTypeOrId, string>
  validationIssues: Record<entityId, string[]>
  pendingSave?: boolean
  conflict?: { serverHash: string; localHash: string }
}
```

### Actions
- `setSteps`, `upsertStep`, `removeStep`, `reorderStep`
- `upsertAction`, `removeAction`, `reorderAction`
- `markDirty(entityId)`
- `computeDesignHash()`
- `setValidationResult({ hash, issues, snapshot })`
- `applyServerSync({ steps, hash })`
- `recordConflict(serverHash, localHash)`

---

## 6. Drag & Drop

Library → Step
- Drag action definition placeholder onto step drop zone triggers action instantiation.
Step reorder
- Sortable vertical list with keyboard accessibility.
Action reorder
- Sortable within a step (no cross-step move initially; future extension).
Framework: `@dnd-kit/core` + `@dnd-kit/sortable`.

Accessibility:
- Custom keyboard handlers: Up/Down to move selection; Alt+Up/Down to reorder.

---

## 7. Parameters & Control Mapping

Mapping:
| Schema Type | Control |
|-------------|---------|
| boolean     | Switch |
| number (bounded) | Slider + numeric display |
| number (unbounded) | Numeric input |
| text short  | Text input |
| text long   | Textarea (expandable) |
| select/enumeration | Select / Combobox |
| multi-select | Tag list (future) |
| json/object | Lazy code editor (future) |

Enhancements:
- Show required indicator
- Inline validation message
- Revert-to-default button if default provided
- Modified badge (dot) for overridden parameters

---

## 8. Validation System

Levels:
1. Structural
   - No empty step names
   - Steps must have valid `type`
   - Conditional/loop must define required condition semantics
2. Parameter
   - Required values present
   - Number bounds enforced
   - Enum membership
3. Semantic
   - No unresolved plugin dependency
   - Unique action IDs and step IDs
   - Loop guard presence (future)
4. Execution Preflight (optional)
   - Detect obviously irrecoverable execution (e.g. empty parallel block)

Severity classification:
- Error: blocks save/compile
- Warning: surfaced but non-blocking
- Info: advisory (e.g. unused parameter)

Store format:
```
validationIssues: {
  [entityId: string]: { severity: "error" | "warning" | "info"; message: string }[]
}
```

Rendering:
- Badge on step/actions with count + highest severity color.
- Panel filter toggles (Errors / Warnings / All).
- Inline icon markers.

---

## 9. Plugin Integration & Drift

Definitions loaded into `ActionRegistry`:
```
{
  id,
  type,
  name,
  category,
  parameters[],
  source: { kind, pluginId?, pluginVersion?, baseActionId? },
  execution?,
  parameterSchemaRaw
}
```

Per-definition signatureHash computed:
`hash(type + category + JSON(parameterSchemaRaw) + execution.transport + baseActionId + pluginVersion)`

Per-action drift logic:
- If action.source.pluginVersion missing in registry → Blocked (hard drift).
- If signatureHash mismatch → Soft drift (allow edit, prompt reconcile).
- Provide “Reconcile” overlay listing changed parameters (added / removed / type changed).

---

## 10. Saving & Versioning

Save triggers:
- Manual (primary save button or Cmd+S)
- Debounced auto-save (idle > 5s, no pending errors)
- Forced save before export

Payload:
```
{
  id,
  visualDesign: { steps, version, lastSaved },
  createSteps: true,
  compileExecution: true
}
```

Version management:
- If structural change (add/remove/reorder step/action or change type) → version bump (auto unless user disables).
- Parameter-only changes may optionally not bump version (configurable toggle; default: no bump).
- UI: dropdown / toggle “Increment version on save”.

Conflict detection:
- Server returns persisted integrityHash.
- If local lastPersistedHash differs and serverHash differs from our pre-save computed hash → conflict modal.
- Provide:
  - View diff summary (step added/removed, action count deltas).
  - Override server (if authorized) or Reload.

---

## 11. Export / Import

Export bundle structure:
```json
{
  "format": "hristudio.design.v1",
  "exportedAt": "...ISO8601...",
  "experiment": {
    "id": "...",
    "name": "...",
    "version": 4,
    "integrityHash": "...",
    "steps": [... canonical steps ...],
    "pluginDependencies": ["pluginA@1.2.1", "robot.voice@0.9.3"]
  },
  "compiled": {
    "graphHash": "...",
    "steps": 12,
    "actions": 47,
    "plan": { /* normalized execution nodes */ }
  }
}
```

Import validation:
- Check `format` signature & version.
- Recompute design hash vs embedded.
- Check plugin availability (list missing).
- Offer “Install required plugins” if privileges allow.

---

## 12. Accessibility & UX Standards

- Keyboard traversal (Tab / Arrow) across steps & actions.
- Focus ring on selected entities.
- ARIA labels on all interactive controls (drag handles, add buttons).
- Color usage passes contrast (WCAG 2.1 AA).
- Non-color indicators for drift (icons / labels).
- All icons: Lucide only.

---

## 13. Performance Considerations

- Virtualize StepFlow when step count > threshold (e.g. 50).
- Memoize parameter forms (avoid re-render on unrelated step changes).
- Incremental hashing—avoid full recompute on each keystroke.
- Lazy-load advanced components (JSON editor, large schema viewers).

---

## 14. Testing Strategy

Unit Tests:
- Hash determinism (same structure -> same hash).
- Hash sensitivity (reorder step / change action type -> different hash).
- Signature drift detection logic.
- Validation rule coverage (structural & parameter).

Integration Tests:
- Save cycle (design → save → fetch → hash equality).
- Plugin dependency missing scenario.
- Conflict resolution workflow.

Edge Cases:
- Empty experiment (0 steps) → validation error.
- Conditional step with no conditions → error.
- Loop step missing guard → warning (future escalation).
- Plugin removed post-design → plugin drift surfaced.

---

## 15. Migration Plan (Internal)

1. Introduce new store + hashing modules.
2. Replace current `BlockDesigner` usage with `DesignerShell`.
3. Port ActionLibrary / StepFlow / PropertiesPanel to new contract.
4. Add SaveBar + drift/UI overlays.
5. Remove deprecated legacy design references (no “enhanced” terminology).
6. Update docs cross-links (`project-overview.md`, `implementation-details.md`).
7. Add export/import UI.
8. Stabilize, then enforce hash validation before trial creation.

---

## 16. Security & Integrity

- Server must recompute its own structural hash during compile to trust design.
- Client-submitted integrityHash considered advisory; never trusted alone.
- Plugin versions pinned explicitly inside actions (no implicit latest resolution).
- Attempting to execute an experiment with unresolved drift prompts required validation.

---

## 17. Future Extensions

| Feature                  | Description |
|--------------------------|-------------|
| Real-time co-editing     | WebSocket presence + granular patch sync |
| Branching workflows      | Conditional branches forming DAG (graph mode) |
| Step templates library   | Shareable reproducible step/action sets |
| Parameter presets        | Save named parameter bundles per action type |
| Timeline estimation view | Aggregate predicted duration |
| Replay / provenance diff | Compare two design versions side-by-side |
| Plugin action migration  | Scripted param transforms on schema changes |
| Execution simulation     | Dry-run graph traversal with timing estimates |

---

## 18. Implementation Checklist (Actionable)

- [ ] hashing.ts (canonical + incremental)
- [ ] validators.ts (structural + param rules)
- [ ] store/useDesignerStore.ts
- [ ] ActionRegistry rewrite with signature hashing
- [ ] ActionLibraryPanel (search, categories, drift indicators)
- [ ] StepFlow + StepCard + ActionItem (DnD with @dnd-kit)
- [ ] PropertiesPanel + ParameterFieldFactory
- [ ] ValidationPanel + badges
- [ ] DependencyInspector + plugin drift mapping
- [ ] SaveBar (dirty, versioning, export)
- [ ] Exporter (JSON bundle) + import hook
- [ ] Conflict modal
- [ ] Drift reconciliation UI
- [ ] Unit & integration tests
- [ ] Docs cross-link updates
- [ ] Remove obsolete legacy code paths

(Track progress in `docs/work_in_progress.md` under “Experiment Designer Redesign Implementation”.)

---

## 19. Cross-References

- See `docs/implementation-details.md` (update with hashing + drift model)
- See `docs/api-routes.md` (`experiments.update`, `experiments.validateDesign`)
- See `docs/database-schema.md` (`steps`, `actions`, provenance fields)
- See `docs/project-overview.md` (designer feature summary)
- See `docs/work_in_progress.md` (status tracking)

---

## 20. Summary

This redesign formalizes a production-grade, reproducible, and extensible experiment design environment with deterministic hashing, plugin-aware action provenance, structured validation, export integrity, and a modular, performance-conscious UI framework. Implementation should now proceed directly against this spec; deviations require documentation updates and justification.

--- 
End of specification.