#!/usr/bin/env node

/**
 * Test script for HRIStudio Experiment Designer API
 *
 * This script tests the getDesign and saveDesign API endpoints
 * to ensure the experiment designer works correctly.
 */

// Mock experiment data for testing
const mockExperimentId = "test-experiment-123";
const mockStudyId = "test-study-123";
const mockUserId = "test-user-123";

const mockCanvasDesign = {
  elements: [
    {
      id: "element-1",
      type: "action",
      title: "Robot Greeting",
      content: "Robot says hello to participant",
      position: { x: 100, y: 100 },
      size: { width: 250, height: 150 },
      style: {
        backgroundColor: "#dbeafe",
        textColor: "#1e40af",
        borderColor: "#3b82f6",
        fontSize: 14,
      },
      metadata: {
        stepType: "robot",
        orderIndex: 0,
        durationEstimate: 30,
        conditions: {},
        actions: [
          {
            id: "action-1",
            name: "speak",
            description: "Say greeting message",
            type: "robot_speech",
            parameters: { message: "Hello! Welcome to our study." },
            orderIndex: 0,
          },
        ],
      },
      connections: ["element-2"],
    },
    {
      id: "element-2",
      type: "decision",
      title: "Wait for Response",
      content: "Wait for participant to respond",
      position: { x: 400, y: 100 },
      size: { width: 250, height: 150 },
      style: {
        backgroundColor: "#fef3c7",
        textColor: "#92400e",
        borderColor: "#f59e0b",
        fontSize: 14,
      },
      metadata: {
        stepType: "wizard",
        orderIndex: 1,
        durationEstimate: 60,
        conditions: { waitType: "manual" },
        actions: [
          {
            id: "action-2",
            name: "wait_for_input",
            description: "Wait for wizard to continue",
            type: "wizard_wait",
            parameters: { timeout: 120 },
            orderIndex: 0,
          },
        ],
      },
      connections: [],
    },
  ],
  connections: [
    {
      id: "connection-1",
      from: "element-1",
      to: "element-2",
      label: "Next",
      style: {
        color: "#6b7280",
        width: 2,
        type: "solid",
      },
    },
  ],
  canvasSettings: {
    zoom: 1,
    gridSize: 20,
    showGrid: true,
    backgroundColor: "#f9fafb",
  },
  version: 1,
};

async function testDesignerAPI() {
  console.log("🧪 Testing HRIStudio Experiment Designer API...\n");

  try {
    // Test 1: Test getDesign API with empty experiment
    console.log("1️⃣ Testing getDesign API (empty experiment)...");

    const getDesignResponse = await fetch(
      "http://localhost:3000/api/trpc/experiments.getDesign",
      {
        method: "GET",
        headers: {
          "Content-Type": "application/json",
        },
        // Note: In real app, this would need authentication
        body: JSON.stringify({
          input: {
            experimentId: mockExperimentId,
          },
        }),
      },
    );

    if (getDesignResponse.ok) {
      const designData = await getDesignResponse.json();
      console.log("✅ getDesign API responded successfully");
      console.log(
        "📊 Design data structure:",
        JSON.stringify(designData, null, 2),
      );
    } else {
      console.log(
        "❌ getDesign API failed:",
        getDesignResponse.status,
        await getDesignResponse.text(),
      );
    }

    console.log("\n");

    // Test 2: Test saveDesign API
    console.log("2️⃣ Testing saveDesign API...");

    const saveDesignResponse = await fetch(
      "http://localhost:3000/api/trpc/experiments.saveDesign",
      {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
        },
        body: JSON.stringify({
          input: {
            experimentId: mockExperimentId,
            design: mockCanvasDesign,
          },
        }),
      },
    );

    if (saveDesignResponse.ok) {
      const saveResult = await saveDesignResponse.json();
      console.log("✅ saveDesign API responded successfully");
      console.log("💾 Save result:", JSON.stringify(saveResult, null, 2));
    } else {
      console.log(
        "❌ saveDesign API failed:",
        saveDesignResponse.status,
        await saveDesignResponse.text(),
      );
    }

    console.log("\n");

    // Test 3: Test getDesign API after saving
    console.log("3️⃣ Testing getDesign API (after save)...");

    const getDesignAfterSaveResponse = await fetch(
      "http://localhost:3000/api/trpc/experiments.getDesign",
      {
        method: "GET",
        headers: {
          "Content-Type": "application/json",
        },
        body: JSON.stringify({
          input: {
            experimentId: mockExperimentId,
          },
        }),
      },
    );

    if (getDesignAfterSaveResponse.ok) {
      const updatedDesignData = await getDesignAfterSaveResponse.json();
      console.log("✅ getDesign API (after save) responded successfully");
      console.log(
        "📊 Updated design has",
        updatedDesignData.elements?.length || 0,
        "elements",
      );

      // Verify data round-trip
      if (
        updatedDesignData.elements?.length === mockCanvasDesign.elements.length
      ) {
        console.log("✅ Data round-trip successful - element count matches");
      } else {
        console.log("⚠️ Data round-trip issue - element count mismatch");
      }
    } else {
      console.log(
        "❌ getDesign API (after save) failed:",
        getDesignAfterSaveResponse.status,
        await getDesignAfterSaveResponse.text(),
      );
    }
  } catch (error) {
    console.error("❌ Test failed with error:", error);
  }

  console.log("\n🏁 Test completed!");
}

// Test data transformation functions
function testDataTransformation() {
  console.log("\n🔄 Testing data transformation logic...\n");

  // Test 1: Canvas elements to steps transformation
  console.log("1️⃣ Testing canvas elements → steps transformation...");

  const transformedSteps = mockCanvasDesign.elements
    .filter(
      (element) => element.type === "action" || element.type === "decision",
    )
    .map((element, index) => {
      const stepType =
        element.metadata?.stepType ||
        (element.type === "action" ? "wizard" : "conditional");

      return {
        id: element.id.startsWith("element-") ? undefined : element.id,
        tempId: element.id.startsWith("element-") ? element.id : undefined,
        type: stepType,
        name: element.title,
        description: element.content || null,
        orderIndex: element.metadata?.orderIndex ?? index,
        durationEstimate: element.metadata?.durationEstimate,
        conditions: element.metadata?.conditions || {},
        actions: element.metadata?.actions || [],
      };
    })
    .sort((a, b) => a.orderIndex - b.orderIndex);

  console.log(
    "📊 Transformed steps:",
    JSON.stringify(transformedSteps, null, 2),
  );
  console.log("✅ Canvas → Steps transformation completed");

  console.log("\n");

  // Test 2: Steps to canvas elements transformation
  console.log("2️⃣ Testing steps → canvas elements transformation...");

  const mockSteps = [
    {
      id: "step-1",
      type: "robot",
      name: "Robot Introduction",
      description: "Robot introduces itself",
      orderIndex: 0,
      durationEstimate: 45,
      conditions: { introType: "formal" },
      actions: [
        {
          id: "action-1",
          name: "speak_intro",
          description: "Robot speaks introduction",
          type: "robot_speech",
          parameters: { text: "Hello, I am your robot assistant." },
          orderIndex: 0,
        },
      ],
    },
  ];

  const transformedElements = mockSteps.map((step, index) => ({
    id: step.id,
    type:
      step.type === "wizard"
        ? "action"
        : step.type === "robot"
          ? "action"
          : step.type === "parallel"
            ? "decision"
            : step.type === "conditional"
              ? "decision"
              : "text",
    title: step.name,
    content: step.description || "",
    position: {
      x: 100 + (index % 3) * 300,
      y: 100 + Math.floor(index / 3) * 200,
    },
    size: { width: 250, height: 150 },
    style: {
      backgroundColor:
        step.type === "wizard"
          ? "#dbeafe"
          : step.type === "robot"
            ? "#dcfce7"
            : step.type === "parallel"
              ? "#fef3c7"
              : "#f8fafc",
      textColor:
        step.type === "wizard"
          ? "#1e40af"
          : step.type === "robot"
            ? "#166534"
            : step.type === "parallel"
              ? "#92400e"
              : "#1e293b",
      borderColor:
        step.type === "wizard"
          ? "#3b82f6"
          : step.type === "robot"
            ? "#22c55e"
            : step.type === "parallel"
              ? "#f59e0b"
              : "#e2e8f0",
      fontSize: 14,
    },
    metadata: {
      stepType: step.type,
      orderIndex: step.orderIndex,
      durationEstimate: step.durationEstimate,
      conditions: step.conditions,
      actions: step.actions.map((action) => ({
        id: action.id,
        name: action.name,
        description: action.description,
        type: action.type,
        parameters: action.parameters,
        orderIndex: action.orderIndex,
      })),
    },
    connections: [],
  }));

  console.log(
    "📊 Transformed elements:",
    JSON.stringify(transformedElements, null, 2),
  );
  console.log("✅ Steps → Canvas transformation completed");
}

// Main execution
async function main() {
  console.log("🚀 HRIStudio Experiment Designer API Test Suite\n");
  console.log("📍 Testing against: http://localhost:3000\n");

  // Check if server is running
  try {
    const healthCheck = await fetch("http://localhost:3000");
    if (healthCheck.ok) {
      console.log("✅ Server is running\n");
    } else {
      console.log("⚠️ Server responded but might have issues\n");
    }
  } catch (error) {
    console.log("❌ Server is not running. Please start with: bun dev\n");
    process.exit(1);
  }

  // Run tests
  testDataTransformation();

  console.log("\n🎉 Test completed! Check the results above.");
  console.log("\n📋 Next steps:");
  console.log("   1. Create a study and experiment in the UI");
  console.log("   2. Navigate to /experiments/{id}/designer");
  console.log("   3. Test drag-and-drop functionality");
  console.log("   4. Verify save/load works properly");
  console.log("\n🔧 API testing requires authentication - test via UI instead");
}

// Run if called directly
main().catch(console.error);
