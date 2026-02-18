
import { describe, it, expect, beforeEach, mock } from "bun:test";
import { TrialExecutionEngine } from "~/server/services/trial-execution";
import type { StepDefinition } from "~/server/services/trial-execution";

// Robust Mock for Drizzle Chaining
const mockQueryExecutor = mock(() => Promise.resolve([]));

const mockBuilder = new Proxy({} as any, {
    get: (target, prop) => {
        if (prop === 'then') {
            return (onfulfilled: any, onrejected: any) => mockQueryExecutor().then(onfulfilled, onrejected);
        }
        // Return self for any chainable method
        return () => mockBuilder;
    }
});

const mockDb = {
    select: mock(() => mockBuilder),
    update: mock(() => mockBuilder),
    insert: mock(() => mockBuilder),
    delete: mock(() => mockBuilder),
    // Helper to mock return values easily
    __setNextResult: (value: any) => mockQueryExecutor.mockResolvedValueOnce(value),
    __reset: () => {
        mockQueryExecutor.mockClear();
        mockQueryExecutor.mockResolvedValue([]); // Default empty
    }
} as any;

// Mock Data
const mockTrialId = "trial-123";
const mockExpId = "exp-123";

const mockStep: StepDefinition = {
    id: "step-1",
    name: "Test Step",
    type: "sequential",
    orderIndex: 0,
    actions: [],
    condition: undefined
};

describe("TrialExecutionEngine", () => {
    let engine: TrialExecutionEngine;

    beforeEach(() => {
        mockDb.__reset();
        engine = new TrialExecutionEngine(mockDb);
    });

    it("should initialize a trial context", async () => {
        // 1. Fetch Trial
        mockDb.__setNextResult([{
            id: mockTrialId,
            experimentId: mockExpId,
            status: "scheduled",
            participantId: "p1"
        }]);

        // 2. Fetch Steps
        mockDb.__setNextResult([]); // Return empty steps for this test

        const context = await engine.initializeTrial(mockTrialId);

        expect(context.trialId).toBe(mockTrialId);
        expect(context.currentStepIndex).toBe(0);
    });

    it("should fail to initialize non-existent trial", async () => {
        mockDb.__setNextResult([]); // No trial found

        const promise = engine.initializeTrial("bad-id");
        // Since we are mocking, we need to ensure the promise rejects as expected
        // The engine throws "Trial bad-id not found"
        expect(promise).rejects.toThrow("not found");
    });
});
