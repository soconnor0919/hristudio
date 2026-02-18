
import { describe, it, expect, beforeEach } from "vitest";
import { createDesignerStore } from "../../../components/experiments/designer/state/store";
import type { ExperimentStep, ExperimentAction } from "../../experiment-designer/types";

// Helper to create a store instance
// We need to bypass the actual hook usage since we are in a non-React env
const createTestStore = () => {
    // Use the exported creator
    return createDesignerStore({
        initialSteps: []
    });
};

describe("Designer Store Integration", () => {
    let store: ReturnType<typeof createTestStore>;

    beforeEach(() => {
        store = createTestStore();
    });

    it("should initialize with empty steps", () => {
        expect(store.getState().steps).toHaveLength(0);
    });

    it("should upsert a new step", () => {
        const step: ExperimentStep = {
            id: "step-1",
            name: "Step 1",
            type: "sequential",
            order: 0,
            trigger: { type: "trial_start", conditions: {} },
            actions: [],
            expanded: true
        };

        store.getState().upsertStep(step);
        expect(store.getState().steps).toHaveLength(1);
        expect(store.getState().steps[0].id).toBe("step-1");
    });

    it("should update an existing step", () => {
        const step: ExperimentStep = {
            id: "step-1",
            name: "Step 1",
            type: "sequential",
            order: 0,
            trigger: { type: "trial_start", conditions: {} },
            actions: [],
            expanded: true
        };
        store.getState().upsertStep(step);

        const updatedStep = { ...step, name: "Updated Step" };
        store.getState().upsertStep(updatedStep);

        expect(store.getState().steps).toHaveLength(1);
        expect(store.getState().steps[0].name).toBe("Updated Step");
    });

    it("should remove a step", () => {
        const step: ExperimentStep = {
            id: "step-1",
            name: "Step 1",
            type: "sequential",
            order: 0,
            trigger: { type: "trial_start", conditions: {} },
            actions: [],
            expanded: true
        };
        store.getState().upsertStep(step);
        store.getState().removeStep(step.id);
        expect(store.getState().steps).toHaveLength(0);
    });

    it("should reorder steps", () => {
        const step1: ExperimentStep = {
            id: "step-1",
            name: "Step 1",
            type: "sequential",
            order: 0,
            trigger: { type: "trial_start", conditions: {} },
            actions: [],
            expanded: true
        };
        const step2: ExperimentStep = {
            id: "step-2",
            name: "Step 2",
            type: "sequential",
            order: 1,
            trigger: { type: "previous_step", conditions: {} },
            actions: [],
            expanded: true
        };

        store.getState().upsertStep(step1);
        store.getState().upsertStep(step2);

        // Move Step 1 to index 1 (swap)
        store.getState().reorderStep(0, 1);

        const steps = store.getState().steps;
        expect(steps[0].id).toBe("step-2");
        expect(steps[1].id).toBe("step-1");

        // Orders should be updated
        expect(steps[0].order).toBe(0);
        expect(steps[1].order).toBe(1);
    });

    it("should upsert an action into a step", () => {
        const step: ExperimentStep = {
            id: "step-1",
            name: "Step 1",
            type: "sequential",
            order: 0,
            trigger: { type: "trial_start", conditions: {} },
            actions: [],
            expanded: true
        };
        store.getState().upsertStep(step);

        const action: ExperimentAction = {
            id: "act-1",
            type: "log",
            name: "Log",
            parameters: {},
            source: { kind: "core", baseActionId: "log" },
            execution: { transport: "internal" }
        };

        store.getState().upsertAction("step-1", action);

        const storedStep = store.getState().steps[0];
        expect(storedStep.actions).toHaveLength(1);
        expect(storedStep.actions[0].id).toBe("act-1");
    });
});
