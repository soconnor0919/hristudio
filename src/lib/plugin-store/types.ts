import { z } from "zod";

// Version compatibility schema
export const versionCompatibilitySchema = z.object({
  hristudio: z.object({
    min: z.string(),
    max: z.string().optional(),
    recommended: z.string().optional(),
  }),
  ros2: z
    .object({
      distributions: z.array(z.string()),
      recommended: z.string().optional(),
    })
    .optional(),
});

// Repository metadata schema
export const storedRepositoryMetadataSchema = z.object({
  id: z.string(),
  urls: z.object({
    git: z.string().url().optional(),
    repository: z.string().url(),
  }),
  trust: z.enum(["official", "verified", "community"]).default("community"),
  enabled: z.boolean().default(true),
  lastSyncedAt: z.date().optional(),
});

export const repositoryMetadataSchema = storedRepositoryMetadataSchema.extend({
  // These fields are fetched from the repository.json but not stored in the database
  name: z.string(),
  description: z.string().optional(),
  official: z.boolean().default(false),
  author: z.object({
    name: z.string(),
    email: z.string().email().optional(),
    url: z.string().url().optional(),
    organization: z.string().optional(),
  }),
  maintainers: z
    .array(
      z.object({
        name: z.string(),
        email: z.string().email().optional(),
        url: z.string().url().optional(),
      }),
    )
    .optional(),
  homepage: z.string().url().optional(),
  license: z.string(),
  defaultBranch: z.string().default("main"),
  lastUpdated: z.string().datetime(),
  compatibility: z.object({
    hristudio: z.object({
      min: z.string(),
      max: z.string().optional(),
      recommended: z.string().optional(),
    }),
    ros2: z
      .object({
        distributions: z.array(z.string()),
        recommended: z.string().optional(),
      })
      .optional(),
  }),
  tags: z.array(z.string()).default([]),
  stats: z
    .object({
      plugins: z.number().default(0),
    })
    .optional(),
  assets: z
    .object({
      icon: z.string().optional(),
      logo: z.string().optional(),
      banner: z.string().optional(),
    })
    .optional(),
});

export type StoredRepositoryMetadata = z.infer<
  typeof storedRepositoryMetadataSchema
>;
export type RepositoryMetadata = z.infer<typeof repositoryMetadataSchema>;

// Core types for the plugin store
export type ActionType =
  | "move" // Robot movement
  | "speak" // Robot speech
  | "wait" // Wait for a duration
  | "input" // Wait for user input
  | "gesture" // Robot gesture
  | "record" // Start/stop recording
  | "condition" // Conditional branching
  | "loop"; // Repeat actions

// Zod schema for parameter properties
export const parameterPropertySchema = z.object({
  type: z.string(),
  title: z.string(),
  description: z.string().optional(),
  default: z.any().optional(),
  minimum: z.number().optional(),
  maximum: z.number().optional(),
  enum: z.array(z.string()).optional(),
  unit: z.string().optional(),
});

// Zod schema for ROS2 QoS settings
export const qosSchema = z.object({
  reliability: z.enum(["reliable", "best_effort"]),
  durability: z.enum(["volatile", "transient_local"]),
  history: z.enum(["keep_last", "keep_all"]),
  depth: z.number().optional(),
});

// Zod schema for action definition
export const actionDefinitionSchema = z.object({
  actionId: z.string(),
  type: z.enum([
    "move",
    "speak",
    "wait",
    "input",
    "gesture",
    "record",
    "condition",
    "loop",
  ]),
  title: z.string(),
  description: z.string(),
  icon: z.string().optional(),
  parameters: z.object({
    type: z.literal("object"),
    properties: z.record(parameterPropertySchema),
    required: z.array(z.string()),
  }),
  ros2: z.object({
    messageType: z.string(),
    topic: z.string().optional(),
    service: z.string().optional(),
    action: z.string().optional(),
    payloadMapping: z.object({
      type: z.enum(["direct", "transform"]),
      map: z.record(z.string()).optional(),
      transformFn: z.string().optional(),
    }),
    qos: qosSchema.optional(),
  }),
});

// Zod schema for the entire robot plugin
export const robotPluginSchema = z.object({
  robotId: z.string(),
  name: z.string(),
  description: z.string().optional(),
  platform: z.string(),
  version: z.string(),

  manufacturer: z.object({
    name: z.string(),
    website: z.string().url().optional(),
    support: z.string().url().optional(),
  }),

  documentation: z.object({
    mainUrl: z.string().url(),
    apiReference: z.string().url().optional(),
    wikiUrl: z.string().url().optional(),
    videoUrl: z.string().url().optional(),
  }),

  assets: z.object({
    thumbnailUrl: z.string(),
    logo: z.string().optional(),
    images: z.object({
      main: z.string(),
      angles: z
        .object({
          front: z.string().optional(),
          side: z.string().optional(),
          top: z.string().optional(),
        })
        .optional(),
      dimensions: z.string().optional(),
    }),
    model: z
      .object({
        format: z.enum(["URDF", "glTF", "other"]),
        url: z.string().url(),
      })
      .optional(),
  }),

  specs: z.object({
    dimensions: z.object({
      length: z.number(),
      width: z.number(),
      height: z.number(),
      weight: z.number(),
    }),
    capabilities: z.array(z.string()),
    maxSpeed: z.number(),
    batteryLife: z.number(),
  }),

  actions: z.array(actionDefinitionSchema),

  ros2Config: z.object({
    namespace: z.string(),
    nodePrefix: z.string(),
    defaultTopics: z.record(z.string()),
  }),
});

// TypeScript types inferred from the Zod schemas
export type ParameterProperty = z.infer<typeof parameterPropertySchema>;
export type QoSSettings = z.infer<typeof qosSchema>;
export type ActionDefinition = z.infer<typeof actionDefinitionSchema>;
export type RobotPlugin = z.infer<typeof robotPluginSchema>;
