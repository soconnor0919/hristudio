# Plugin Store System

## Overview

The Plugin Store is a core feature of HRIStudio that manages robot plugins, their repositories, and their integration into the platform. It provides a robust system for loading, validating, and utilizing robot plugins within experiments.

## Architecture

### Core Components

```typescript
export class PluginStore {
  private plugins: Map<string, RobotPlugin> = new Map();
  private repositories: Map<string, RepositoryMetadata> = new Map();
  private transformFunctions: Map<string, Function> = new Map();
  private pluginToRepo: Map<string, string> = new Map();
  private lastRefresh: Map<string, number> = new Map();
  private readonly CACHE_TTL = 5 * 60 * 1000; // 5 minutes
}
```

### Plugin Types

```typescript
interface RobotPlugin {
  robotId: string;
  name: string;
  description: string;
  platform: string;
  version: string;
  manufacturer: {
    name: string;
    website?: string;
    support?: string;
  };
  documentation: {
    mainUrl: string;
    apiReference?: string;
    wikiUrl?: string;
    videoUrl?: string;
  };
  assets: {
    thumbnailUrl: string;
    images: {
      main: string;
      angles?: {
        front?: string;
        side?: string;
        top?: string;
      };
      dimensions?: string;
    };
    model?: {
      format: string;
      url: string;
    };
  };
  specs: {
    dimensions: {
      length: number;
      width: number;
      height: number;
      weight: number;
    };
    capabilities: string[];
    maxSpeed: number;
    batteryLife: number;
  };
  actions: ActionDefinition[];
}
```

## Repository Management

### Loading Repositories

```typescript
async loadRepository(url: string): Promise<RepositoryMetadata> {
  // Clean URL
  const cleanUrl = url.trim().replace(/\/$/, "");
  
  try {
    // Fetch repository metadata
    const metadataUrl = this.getRepositoryFileUrl(cleanUrl, "repository.json");
    const response = await fetch(metadataUrl);
    
    if (!response.ok) {
      throw new Error(`Failed to fetch repository metadata: ${response.statusText}`);
    }
    
    const metadata = await response.json();
    // Validate and process metadata
    return metadata;
  } catch (error) {
    throw new PluginLoadError("Failed to load repository", undefined, error);
  }
}
```

### Repository Metadata

```typescript
interface RepositoryMetadata {
  id: string;
  name: string;
  description: string;
  url: string;
  official: boolean;
  author: {
    name: string;
    email?: string;
    url?: string;
    organization?: string;
  };
  maintainers?: Array<{
    name: string;
    url?: string;
  }>;
  compatibility: {
    hristudio: {
      min: string;
      recommended?: string;
    };
    ros2?: {
      distributions: string[];
      recommended?: string;
    };
  };
  stats: {
    downloads: number;
    stars: number;
    plugins: number;
  };
}
```

## Plugin Loading & Validation

### Loading Process

1. **Repository Metadata:**
   ```typescript
   private async loadRepositoryPlugins(repository: RepositoryMetadata) {
     const metadataUrl = this.getRepositoryFileUrl(
       repository.url,
       "repository.json"
     );
     // Fetch and validate metadata
   }
   ```

2. **Individual Plugins:**
   ```typescript
   async loadPluginFromJson(jsonString: string): Promise<RobotPlugin> {
     try {
       const data = JSON.parse(jsonString);
       return await this.validatePlugin(data);
     } catch (error) {
       throw new PluginLoadError(
         "Failed to parse plugin JSON",
         undefined,
         error
       );
     }
   }
   ```

### Validation

```typescript
private async validatePlugin(data: unknown): Promise<RobotPlugin> {
  try {
    return robotPluginSchema.parse(data);
  } catch (error) {
    if (error instanceof z.ZodError) {
      throw new PluginLoadError(
        `Invalid plugin format: ${error.errors.map(e => e.message).join(", ")}`,
        undefined,
        error
      );
    }
    throw error;
  }
}
```

## Action System

### Action Types

```typescript
type ActionType = 
  | "move"      // Robot movement
  | "speak"     // Robot speech
  | "wait"      // Wait for a duration
  | "input"     // Wait for user input
  | "gesture"   // Robot gesture
  | "record"    // Start/stop recording
  | "condition" // Conditional branching
  | "loop";     // Repeat actions
```

### Action Definition

```typescript
interface ActionDefinition {
  actionId: string;
  type: ActionType;
  title: string;
  description: string;
  parameters: {
    type: "object";
    properties: Record<string, ParameterProperty>;
    required: string[];
  };
  ros2?: {
    messageType: string;
    topic?: string;
    service?: string;
    action?: string;
    payloadMapping: {
      type: "direct" | "transform";
      transformFn?: string;
    };
    qos?: QoSSettings;
  };
}
```

### Transform Functions

```typescript
private transformToTwist(params: { linear: number; angular: number }) {
  return {
    linear: {
      x: params.linear,
      y: 0.0,
      z: 0.0
    },
    angular: {
      x: 0.0,
      y: 0.0,
      z: params.angular
    }
  };
}
```

## Caching & Performance

### Cache Implementation

```typescript
private shouldRefreshCache(repositoryId: string): boolean {
  const lastRefresh = this.lastRefresh.get(repositoryId);
  if (!lastRefresh) return true;
  
  const now = Date.now();
  return now - lastRefresh > this.CACHE_TTL;
}
```

### Error Handling

```typescript
export class PluginLoadError extends Error {
  constructor(
    message: string,
    public robotId?: string,
    public cause?: unknown
  ) {
    super(message);
    this.name = "PluginLoadError";
  }
}
```

## Usage Examples

### Loading a Repository

```typescript
const store = new PluginStore();
await store.loadRepository("https://github.com/org/robot-plugins");
```

### Getting Plugin Information

```typescript
const plugin = store.getPlugin("turtlebot3-burger");
if (plugin) {
  console.log(`Loaded ${plugin.name} version ${plugin.version}`);
  console.log(`Supported actions: ${plugin.actions.length}`);
}
```

### Registering Transform Functions

```typescript
store.registerTransformFunction("transformToTwist", (params) => {
  // Custom transformation logic
  return transformedData;
});
```

## Best Practices

1. **Error Handling:**
   - Always catch and properly handle plugin loading errors
   - Provide meaningful error messages
   - Include error context when possible

2. **Validation:**
   - Validate all plugin metadata
   - Verify action parameters
   - Check compatibility requirements

3. **Performance:**
   - Use caching appropriately
   - Implement lazy loading where possible
   - Monitor memory usage

4. **Security:**
   - Validate URLs and file paths
   - Implement proper access controls
   - Sanitize plugin inputs

## Future Enhancements

1. **Plugin Versioning:**
   - Semantic versioning support
   - Version compatibility checking
   - Update management

2. **Advanced Caching:**
   - Persistent cache storage
   - Cache invalidation strategies
   - Partial cache updates

3. **Plugin Marketplace:**
   - User ratings and reviews
   - Download statistics
   - Community contributions

4. **Enhanced Validation:**
   - Runtime validation
   - Performance benchmarking
   - Compatibility testing

## Dynamic Plugin Loading

The plugin store in HRI Studio supports modular loading of robot actions. Not every robot action is installed by default; instead, only the necessary plugins for the desired robots are installed. This approach offers several benefits:

- Flexibility: Deploy only the robots and actions you need.
- Performance: Avoid loading unnecessary modules, leading to faster startup times and reduced memory usage.
- Extensibility: Allow companies and users to host their own plugin repositories with custom robot actions.

### Implementation Details

1. Each plugin should export a manifest adhering to the `RobotPlugin` interface, containing a unique identifier, display name, and a list of actions.
2. The system loads only the configured plugins, which can be managed via environment variables, a database table, or an admin interface.
3. Dynamic imports are used in the Next.js server environment to load robot actions on demand. For example:

```ts
async function loadPlugin(pluginUrl: string): Promise<RobotPlugin> {
  const pluginModule = await import(pluginUrl);
  return pluginModule.default as RobotPlugin;
}
```

This design ensures that HRI Studio remains lean and agile, seamlessly integrating new robot actions without overhead. 