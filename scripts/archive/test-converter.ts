
import { convertDatabaseToAction } from "../../src/lib/experiment-designer/block-converter";

const mockDbAction = {
    id: "eaf8f85b-75cf-4973-b436-092516b4e0e4",
    name: "Introduction Sequence",
    description: null,
    type: "sequence",
    orderIndex: 0,
    parameters: {
        "children": [
            {
                "id": "75018b01-a964-41fb-8612-940a29020d4a",
                "name": "Say Hello",
                "type": "nao6-ros2.say_text",
                "category": "interaction",
                "parameters": {
                    "text": "Hello there!"
                }
            },
            {
                "id": "d7020530-6477-41f3-84a4-5141778c93da",
                "name": "Wave Hand",
                "type": "nao6-ros2.move_arm",
                "category": "movement",
                "parameters": {
                    "arm": "right",
                    "action": "wave"
                }
            }
        ]
    },
    timeout: null,
    retryCount: 0,
    sourceKind: "core",
    pluginId: "hristudio-core",
    pluginVersion: null,
    robotId: null,
    baseActionId: null,
    category: "control",
    transport: null,
    ros2: null,
    rest: null,
    retryable: null,
    parameterSchemaRaw: null
};

console.log("Testing convertDatabaseToAction...");
try {
    const result = convertDatabaseToAction(mockDbAction);
    console.log("Result:", JSON.stringify(result, null, 2));

    if (result.children && result.children.length > 0) {
        console.log("✅ Children hydrated successfully.");
    } else {
        console.error("❌ Children NOT hydrated.");
    }
} catch (e) {
    console.error("❌ Error during conversion:", e);
}
