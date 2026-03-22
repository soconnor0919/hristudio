# HRIStudio Plugin System Implementation Guide

## Overview

This guide provides step-by-step instructions for implementing the HRIStudio Plugin System integration. You have access to two repositories:

1. **HRIStudio Main Repository** - Contains the core platform
2. **Plugin Repository** - Contains robot plugin definitions and web interface

Your task is to create a plugin store within HRIStudio and modify the plugin repository to ensure seamless integration.

## Architecture Overview

```
HRIStudio Platform
├── Plugin Store (Frontend)
├── Plugin Manager (Backend)
├── Plugin Registry (Database)
└── ROS2 Integration Layer
    └── Plugin Repository (External)
        ├── Repository Metadata
        ├── Plugin Definitions
        └── Web Interface
```

## Phase 1: Plugin Store Frontend Implementation

### 1.1 Create Plugin Store Page

**Location**: `src/app/(dashboard)/plugins/page.tsx`

Create a new page that displays available plugins from registered repositories.

```typescript
// Key features to implement:
- Repository management (add/remove plugin repositories)
- Plugin browsing with categories and search
- Plugin details modal/page
- Installation status tracking
- Trust level indicators (Official, Verified, Community)
```

**UI Requirements**:
- Use existing HRIStudio design system (shadcn/ui)
- Follow established patterns from studies/experiments pages
- Include plugin cards with thumbnails, descriptions, and metadata
- Implement filtering by category, platform (ROS2), trust level
- Add search functionality

### 1.2 Plugin Repository Management

**Location**: `src/components/plugins/repository-manager.tsx`

```typescript
// Features to implement:
- Add repository by URL
- Validate repository structure
- Display repository metadata (name, trust level, plugin count)
- Enable/disable repositories
- Remove repositories
- Repository status indicators (online, offline, error)
```

### 1.3 Plugin Installation Interface

**Location**: `src/components/plugins/plugin-installer.tsx`

```typescript
// Features to implement:
- Plugin installation progress
- Dependency checking
- Version compatibility validation
- Installation success/error handling
- Plugin configuration interface
```

## Phase 2: Plugin Manager Backend Implementation

### 2.1 Database Schema Extensions

**Location**: `src/server/db/schema/plugins.ts`

Add these tables to the existing schema:

```sql
-- Plugin repositories
CREATE TABLE plugin_repositories (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  name VARCHAR(255) NOT NULL,
  url TEXT NOT NULL UNIQUE,
  trust_level VARCHAR(20) NOT NULL CHECK (trust_level IN ('official', 'verified', 'community')),
  enabled BOOLEAN DEFAULT true,
  last_synced TIMESTAMP,
  metadata JSONB DEFAULT '{}',
  created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- Installed plugins
CREATE TABLE installed_plugins (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  repository_id UUID NOT NULL REFERENCES plugin_repositories(id) ON DELETE CASCADE,
  plugin_id VARCHAR(255) NOT NULL, -- robotId from plugin definition
  name VARCHAR(255) NOT NULL,
  version VARCHAR(50) NOT NULL,
  configuration JSONB DEFAULT '{}',
  enabled BOOLEAN DEFAULT true,
  installed_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  installed_by UUID NOT NULL REFERENCES users(id),
  UNIQUE(repository_id, plugin_id)
);

-- Plugin usage in studies
CREATE TABLE study_plugins (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  study_id UUID NOT NULL REFERENCES studies(id) ON DELETE CASCADE,
  installed_plugin_id UUID NOT NULL REFERENCES installed_plugins(id),
  configuration JSONB DEFAULT '{}',
  created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  UNIQUE(study_id, installed_plugin_id)
);
```

### 2.2 tRPC Routes Implementation

**Location**: `src/server/api/routers/plugins.ts`

```typescript
export const pluginsRouter = createTRPCRouter({
  // Repository management
  addRepository: protectedProcedure
    .input(z.object({
      url: z.string().url(),
      name: z.string().optional()
    }))
    .mutation(async ({ ctx, input }) => {
      // Validate repository structure
      // Add to database
      // Sync plugins
    }),

  listRepositories: protectedProcedure
    .query(async ({ ctx }) => {
      // Return user's accessible repositories
    }),

  syncRepository: protectedProcedure
    .input(z.object({ repositoryId: z.string().uuid() }))
    .mutation(async ({ ctx, input }) => {
      // Fetch repository.json
      // Update plugin definitions
      // Handle errors
    }),

  // Plugin management
  listAvailablePlugins: protectedProcedure
    .input(z.object({
      repositoryId: z.string().uuid().optional(),
      search: z.string().optional(),
      category: z.string().optional(),
      platform: z.string().optional()
    }))
    .query(async ({ ctx, input }) => {
      // Fetch plugins from repositories
      // Apply filters
      // Return plugin metadata
    }),

  installPlugin: protectedProcedure
    .input(z.object({
      repositoryId: z.string().uuid(),
      pluginId: z.string(),
      configuration: z.record(z.any()).optional()
    }))
    .mutation(async ({ ctx, input }) => {
      // Validate plugin compatibility
      // Install plugin
      // Create plugin instance
    }),

  listInstalledPlugins: protectedProcedure
    .query(async ({ ctx }) => {
      // Return user's installed plugins
    }),

  getPluginActions: protectedProcedure
    .input(z.object({ pluginId: z.string() }))
    .query(async ({ ctx, input }) => {
      // Return plugin action definitions
      // For use in experiment designer
    })
});
```

### 2.3 Plugin Registry Service

**Location**: `src/lib/plugins/registry.ts`

```typescript
export class PluginRegistry {
  // Fetch and validate repository metadata
  async fetchRepository(url: string): Promise<RepositoryMetadata>
  
  // Sync plugins from repository
  async syncRepository(repositoryId: string): Promise<void>
  
  // Load plugin definition
  async loadPlugin(repositoryId: string, pluginId: string): Promise<PluginDefinition>
  
  // Validate plugin compatibility
  async validatePlugin(plugin: PluginDefinition): Promise<ValidationResult>
  
  // Install plugin
  async installPlugin(repositoryId: string, pluginId: string, config?: any): Promise<void>
}
```

## Phase 3: Plugin Repository Modifications

### 3.1 Schema Enhancements

**Location**: Plugin Repository - `docs/schema.md`

Update the plugin schema to include HRIStudio-specific fields:

```json
{
  "robotId": "string (required)",
  "name": "string (required)", 
  "description": "string (optional)",
  "platform": "string (required)",
  "version": "string (required)",
  
  // Add these HRIStudio-specific fields:
  "pluginApiVersion": "string (required) - Plugin API version",
  "hriStudioVersion": "string (required) - Minimum HRIStudio version", 
  "trustLevel": "string (enum: official|verified|community)",
  "category": "string (required) - Plugin category for UI organization",
  
  // Enhanced action schema:
  "actions": [
    {
      "id": "string (required) - Unique action identifier",
      "name": "string (required) - Display name",
      "description": "string (optional)",
      "category": "string (required) - movement|interaction|sensors|logic",
      "icon": "string (optional) - Lucide icon name",
      "timeout": "number (optional) - Default timeout in milliseconds",
      "retryable": "boolean (optional) - Can this action be retried on failure",
      
      "parameterSchema": {
        "type": "object",
        "properties": {
          // Zod-compatible parameter definitions
        },
        "required": ["array of required parameter names"]
      },
      
      "ros2": {
        // Existing ROS2 configuration
      }
    }
  ]
}
```

### 3.2 TurtleBot3 Plugin Update

**Location**: Plugin Repository - `plugins/turtlebot3-burger.json`

Add the missing HRIStudio fields to the existing plugin:

```json
{
  "robotId": "turtlebot3-burger",
  "name": "TurtleBot3 Burger",
  "description": "A compact, affordable, programmable, ROS2-based mobile robot for education and research",
  "platform": "ROS2",
  "version": "2.0.0",
  
  // Add these new fields:
  "pluginApiVersion": "1.0",
  "hriStudioVersion": ">=0.1.0",
  "trustLevel": "official",
  "category": "mobile-robot",
  
  // Update actions with HRIStudio fields:
  "actions": [
    {
      "id": "move_velocity", // Changed from actionId
      "name": "Set Velocity", // Changed from title
      "description": "Control the robot's linear and angular velocity",
      "category": "movement", // New field
      "icon": "navigation", // New field
      "timeout": 30000, // New field
      "retryable": true, // New field
      
      "parameterSchema": {
        // Convert existing parameters to HRIStudio format
        "type": "object",
        "properties": {
          "linear": {
            "type": "number",
            "minimum": -0.22,
            "maximum": 0.22,
            "default": 0,
            "description": "Forward/backward velocity in m/s"
          },
          "angular": {
            "type": "number",
            "minimum": -2.84,
            "maximum": 2.84,
            "default": 0,
            "description": "Rotational velocity in rad/s"
          }
        },
        "required": ["linear", "angular"]
      },
      
      // Keep existing ros2 config
      "ros2": {
        "messageType": "geometry_msgs/msg/Twist",
        "topic": "/cmd_vel",
        "payloadMapping": {
          "type": "transform",
          "transformFn": "transformToTwist"
        },
        "qos": {
          "reliability": "reliable",
          "durability": "volatile",
          "history": "keep_last",
          "depth": 1
        }
      }
    }
  ]
}
```

### 3.3 Repository Metadata Update

**Location**: Plugin Repository - `repository.json`

Add HRIStudio-specific metadata:

```json
{
  "id": "hristudio-official",
  "name": "HRIStudio Official Robot Plugins",
  "description": "Official collection of robot plugins maintained by the HRIStudio team",
  
  // Add API versioning:
  "apiVersion": "1.0",
  "pluginApiVersion": "1.0",
  
  // Add plugin categories:
  "categories": [
    {
      "id": "mobile-robots",
      "name": "Mobile Robots", 
      "description": "Wheeled and tracked mobile platforms"
    },
    {
      "id": "manipulators",
      "name": "Manipulators",
      "description": "Robotic arms and end effectors"
    },
    {
      "id": "humanoids", 
      "name": "Humanoid Robots",
      "description": "Human-like robots for social interaction"
    },
    {
      "id": "drones",
      "name": "Aerial Vehicles",
      "description": "Quadcopters and fixed-wing UAVs"
    }
  ],
  
  // Keep existing fields...
  "compatibility": {
    "hristudio": {
      "min": "0.1.0",
      "recommended": "0.1.0"
    },
    "ros2": {
      "distributions": ["humble", "iron"],
      "recommended": "iron"
    }
  }
}
```

## Phase 4: Integration Implementation

### 4.1 Experiment Designer Integration

**Location**: HRIStudio - `src/components/experiments/designer/EnhancedBlockDesigner.tsx`

Add plugin-based action loading to the block designer:

```typescript
// In the block registry, load actions from installed plugins:
const loadPluginActions = async (studyId: string) => {
  const installedPlugins = await api.plugins.getStudyPlugins.query({ studyId });
  
  for (const plugin of installedPlugins) {
    const actions = await api.plugins.getPluginActions.query({ 
      pluginId: plugin.id 
    });
    
    // Register actions in block registry
    actions.forEach(action => {
      blockRegistry.register({
        id: `${plugin.id}.${action.id}`,
        name: action.name,
        description: action.description,
        category: action.category,
        icon: action.icon || 'bot',
        shape: 'action',
        color: getCategoryColor(action.category),
        parameters: convertToZodSchema(action.parameterSchema),
        metadata: {
          pluginId: plugin.id,
          robotId: plugin.robotId,
          ros2Config: action.ros2
        }
      });
    });
  }
};
```

### 4.2 Trial Execution Integration

**Location**: HRIStudio - `src/lib/plugins/execution.ts`

Create plugin execution interface:

```typescript
export class PluginExecutor {
  private installedPlugins = new Map<string, InstalledPlugin>();
  private rosConnections = new Map<string, RosConnection>();

  async executePluginAction(
    pluginId: string, 
    actionId: string, 
    parameters: Record<string, any>
  ): Promise<ActionResult> {
    const plugin = this.installedPlugins.get(pluginId);
    if (!plugin) {
      throw new Error(`Plugin ${pluginId} not found`);
    }

    const action = plugin.actions.find(a => a.id === actionId);
    if (!action) {
      throw new Error(`Action ${actionId} not found in plugin ${pluginId}`);
    }

    // Validate parameters against schema
    const validation = this.validateParameters(action.parameterSchema, parameters);
    if (!validation.success) {
      throw new Error(`Parameter validation failed: ${validation.error}`);
    }

    // Execute via ROS2 if configured
    if (action.ros2) {
      return this.executeRos2Action(plugin, action, parameters);
    }

    // Execute via REST API if configured
    if (action.rest) {
      return this.executeRestAction(plugin, action, parameters);
    }

    throw new Error(`No execution method configured for action ${actionId}`);
  }

  private async executeRos2Action(
    plugin: InstalledPlugin,
    action: PluginAction, 
    parameters: Record<string, any>
  ): Promise<ActionResult> {
    const connection = this.getRosConnection(plugin.id);
    
    // Transform parameters according to payload mapping
    const payload = this.transformPayload(action.ros2.payloadMapping, parameters);
    
    // Publish to topic or call service
    if (action.ros2.topic) {
      return this.publishToTopic(connection, action.ros2, payload);
    } else if (action.ros2.service) {
      return this.callService(connection, action.ros2, payload);
    } else if (action.ros2.action) {
      return this.executeAction(connection, action.ros2, payload);
    }
    
    throw new Error('No ROS2 communication method specified');
  }
}
```

### 4.3 Plugin Store Navigation

**Location**: HRIStudio - `src/components/layout/navigation/SidebarNav.tsx`

Add plugin store to the navigation:

```typescript
const navigationItems = [
  {
    title: "Dashboard",
    href: "/",
    icon: LayoutDashboard,
    description: "Overview and quick actions"
  },
  {
    title: "Studies", 
    href: "/studies",
    icon: FolderOpen,
    description: "Research projects and team collaboration"
  },
  {
    title: "Experiments",
    href: "/experiments", 
    icon: FlaskConical,
    description: "Protocol design and validation"
  },
  {
    title: "Participants",
    href: "/participants",
    icon: Users,
    description: "Participant management and consent"
  },
  {
    title: "Trials",
    href: "/trials",
    icon: Play,
    description: "Experiment execution and monitoring"
  },
  // Add plugin store:
  {
    title: "Plugin Store",
    href: "/plugins",
    icon: Package,
    description: "Robot plugins and integrations"
  },
  {
    title: "Admin",
    href: "/admin",
    icon: Settings,
    description: "System administration",
    roles: ["administrator"]
  }
];
```

### 4.4 Plugin Configuration in Studies

**Location**: HRIStudio - `src/app/(dashboard)/studies/[studyId]/settings/page.tsx`

Add plugin configuration to study settings:

```typescript
const StudySettingsPage = ({ studyId }: { studyId: string }) => {
  const installedPlugins = api.plugins.listInstalledPlugins.useQuery();
  const studyPlugins = api.plugins.getStudyPlugins.useQuery({ studyId });
  
  return (
    <PageLayout title="Study Settings">
      <Tabs defaultValue="general">
        <TabsList>
          <TabsTrigger value="general">General</TabsTrigger>
          <TabsTrigger value="team">Team</TabsTrigger>
          <TabsTrigger value="plugins">Robot Plugins</TabsTrigger>
          <TabsTrigger value="permissions">Permissions</TabsTrigger>
        </TabsList>
        
        <TabsContent value="plugins">
          <Card>
            <CardHeader>
              <CardTitle>Robot Plugins</CardTitle>
              <CardDescription>
                Configure which robot plugins are available for this study
              </CardDescription>
            </CardHeader>
            <CardContent>
              <PluginConfiguration
                studyId={studyId}
                availablePlugins={installedPlugins.data || []}
                enabledPlugins={studyPlugins.data || []}
              />
            </CardContent>
          </Card>
        </TabsContent>
      </Tabs>
    </PageLayout>
  );
};
```

## Phase 5: Testing and Validation

### 5.1 Plugin Repository Testing

Create test scripts to validate:
- Repository structure and schema compliance
- Plugin definition validation
- Web interface functionality
- API endpoint responses

### 5.2 HRIStudio Integration Testing

Test the complete flow:
1. Add plugin repository to HRIStudio
2. Install a plugin from the repository
3. Configure plugin for a study
4. Use plugin actions in experiment designer
5. Execute plugin actions during trial

### 5.3 End-to-End Testing

Create automated tests that:
- Validate plugin installation process
- Test ROS2 communication via rosbridge
- Verify parameter validation and transformation
- Test error handling and recovery

## Deployment Checklist

### Plugin Repository
- [ ] Update plugin schema documentation
- [ ] Enhance existing plugin definitions
- [ ] Test web interface with new schema
- [ ] Deploy to GitHub Pages or hosting platform
- [ ] Validate HTTPS access and CORS headers

### HRIStudio Platform
- [ ] Implement database schema migrations
- [ ] Create plugin store frontend pages
- [ ] Implement plugin management tRPC routes
- [ ] Integrate plugins with experiment designer
- [ ] Add plugin execution to trial system
- [ ] Update navigation to include plugin store
- [ ] Add plugin configuration to study settings

### Integration Testing
- [ ] Test repository discovery and syncing
- [ ] Validate plugin installation workflow
- [ ] Test plugin action execution
- [ ] Verify ROS2 integration works end-to-end
- [ ] Test error handling and user feedback

This implementation will create a complete plugin ecosystem for HRIStudio, allowing researchers to easily discover, install, and use robot plugins in their studies.