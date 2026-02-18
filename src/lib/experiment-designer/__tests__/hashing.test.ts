
import { describe, it, expect } from "vitest";
import { Hashing } from "../../../components/experiments/designer/state/hashing";
import type { ExperimentStep, ExperimentAction } from "../../experiment-designer/types";

describe("Hashing Utilities", () => {
    describe("canonicalize", () => {
        it("should sort object keys", () => {
            const obj1 = { b: 2, a: 1 };
            const obj2 = { a: 1, b: 2 };
            expect(JSON.stringify(Hashing.canonicalize(obj1)))
                .toBe(JSON.stringify(Hashing.canonicalize(obj2)));
        });

        it("should remove undefined values", () => {
            const obj = { a: 1, b: undefined, c: null };
            const canonical = Hashing.canonicalize(obj) as any;
            expect(canonical).toHaveProperty("a");
            expect(canonical).toHaveProperty("c"); // null is preserved
            expect(canonical).not.toHaveProperty("b");
        });

        it("should preserve array order", () => {
            const arr = [3, 1, 2];
            const canonical = Hashing.canonicalize(arr);
            expect(canonical).toEqual([3, 1, 2]);
        });
    });

    describe("computeDesignHash", () => {
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

        it("should produce deterministic hash regardless of input array order", async () => {
            const hash1 = await Hashing.computeDesignHash([step1, step2]);
            const hash2 = await Hashing.computeDesignHash([step2, step1]);
            expect(hash1).toBe(hash2);
        });

        it("should change hash when step content changes", async () => {
            const hash1 = await Hashing.computeDesignHash([step1]);
            const modifiedStep = { ...step1, name: "Modified Name" };
            const hash2 = await Hashing.computeDesignHash([modifiedStep]);
            expect(hash1).not.toBe(hash2);
        });

        it("should change hash when parameters change if included", async () => {
            const action: ExperimentAction = {
                id: "act-1",
                type: "log",
                name: "Log",
                parameters: { message: "A" },
                source: { kind: "core", baseActionId: "log" },
                execution: { transport: "internal" }
            };
            const stepWithAction = { ...step1, actions: [action] };

            const hash1 = await Hashing.computeDesignHash([stepWithAction], { includeParameterValues: true });

            const modifiedAction = { ...action, parameters: { message: "B" } };
            const stepModified = { ...step1, actions: [modifiedAction] };

            const hash2 = await Hashing.computeDesignHash([stepModified], { includeParameterValues: true });

            expect(hash1).not.toBe(hash2);
        });

        it("should NOT change hash when parameters change if excluded", async () => {
            const action: ExperimentAction = {
                id: "act-1",
                type: "log",
                name: "Log",
                parameters: { message: "A" },
                source: { kind: "core", baseActionId: "log" },
                execution: { transport: "internal" }
            };
            const stepWithAction = { ...step1, actions: [action] };

            const hash1 = await Hashing.computeDesignHash([stepWithAction], { includeParameterValues: false });

            const modifiedAction = { ...action, parameters: { message: "B" } };
            const stepModified = { ...step1, actions: [modifiedAction] };

            const hash2 = await Hashing.computeDesignHash([stepModified], { includeParameterValues: false });

            expect(hash1).toBe(hash2);
        });
    });
});
