import { z } from "zod";
import { type RobotPlugin, type RepositoryMetadata, type StoredRepositoryMetadata, robotPluginSchema, repositoryMetadataSchema, storedRepositoryMetadataSchema } from "./types";
import { db } from "~/server/db";
import { pluginRepositories } from "~/server/db/schema/store";
import { eq } from "drizzle-orm";

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

export class PluginStore {
  private plugins: Map<string, RobotPlugin> = new Map();
  private repositories: Map<string, RepositoryMetadata> = new Map();
  private transformFunctions: Map<string, Function> = new Map();
  private pluginToRepo: Map<string, string> = new Map(); // Maps plugin IDs to repository IDs
  private lastRefresh: Map<string, number> = new Map(); // Cache timestamps
  private readonly CACHE_TTL = 5 * 60 * 1000; // 5 minutes

  constructor() {
    // Register built-in transform functions
    this.registerTransformFunction("transformToTwist", this.transformToTwist);
    this.registerTransformFunction("transformToPoseStamped", this.transformToPoseStamped);
  }

  private getRepositoryFileUrl(baseUrl: string, filePath: string): string {
    try {
      // Clean URLs and join them
      const cleanBaseUrl = baseUrl.replace(/\/$/, '');
      const cleanFilePath = filePath.replace(/^\//, '');
      return `${cleanBaseUrl}/${cleanFilePath}`;
    } catch (error) {
      console.error('Failed to construct repository file URL:', error);
      throw error;
    }
  }

  async initialize() {
    try {
      // Load repositories from database
      const dbRepositories = await db.query.pluginRepositories.findMany();
      
      for (const repository of dbRepositories) {
        if (!repository.enabled) continue;

        // Convert database model to repository metadata
        const storedMetadata: StoredRepositoryMetadata = {
          id: repository.id,
          url: repository.url,
          trust: repository.trust as "official" | "verified" | "community",
          enabled: repository.enabled,
          lastSyncedAt: repository.lastSyncedAt ?? undefined,
        };
        
        try {
          // Fetch full metadata from repository
          const metadata = await this.refreshRepositoryMetadata(storedMetadata);
          
          // Add to in-memory cache
          this.repositories.set(repository.id, metadata);
          
          // Always load plugins on initialization
          await this.loadRepositoryPlugins(metadata);
          this.lastRefresh.set(repository.id, Date.now());

          // Update last synced timestamp
          await db.update(pluginRepositories)
            .set({ 
              lastSyncedAt: new Date(),
              updatedAt: new Date(),
            })
            .where(eq(pluginRepositories.id, repository.id));
        } catch (error) {
          console.warn(`Failed to refresh repository metadata for ${repository.id}:`, error);
          // Continue with next repository if refresh fails
        }
      }
    } catch (error) {
      console.error("Failed to initialize plugin store:", error);
      throw new PluginLoadError(
        "Failed to initialize plugin store",
        undefined,
        error
      );
    }
  }

  private shouldRefreshCache(repositoryId: string): boolean {
    const lastRefreshTime = this.lastRefresh.get(repositoryId);
    if (!lastRefreshTime) return true;
    return Date.now() - lastRefreshTime > this.CACHE_TTL;
  }

  private async refreshRepositoryMetadata(repository: StoredRepositoryMetadata): Promise<RepositoryMetadata> {
    try {
      const repoUrl = this.getRepositoryFileUrl(repository.url, "repository.json");
      const response = await fetch(repoUrl);
      
      if (!response.ok) {
        throw new Error(`Failed to fetch repository metadata: ${response.statusText}`);
      }

      const text = await response.text();
      if (!text) {
        throw new Error("Empty response from repository");
      }

      const data = JSON.parse(text);
      const metadata = await repositoryMetadataSchema.parseAsync({
        ...data,
        id: repository.id,
        enabled: repository.enabled,
        lastSyncedAt: repository.lastSyncedAt,
      });

      // Transform asset URLs to absolute URLs
      if (metadata.assets) {
        metadata.assets = {
          icon: metadata.assets.icon ? this.getRepositoryFileUrl(repository.url, metadata.assets.icon) : undefined,
          logo: metadata.assets.logo ? this.getRepositoryFileUrl(repository.url, metadata.assets.logo) : undefined,
          banner: metadata.assets.banner ? this.getRepositoryFileUrl(repository.url, metadata.assets.banner) : undefined,
        };
      }

      // Initialize stats with default values
      metadata.stats = {
        downloads: 0,
        stars: 0,
        plugins: 0,
        ...metadata.stats,
      };

      // Update in-memory cache
      this.repositories.set(repository.id, metadata);
      this.lastRefresh.set(repository.id, Date.now());

      return metadata;
    } catch (error) {
      console.error(`Failed to refresh repository metadata for ${repository.id}:`, error);
      throw error;
    }
  }

  async loadRepository(url: string): Promise<RepositoryMetadata> {
    try {
      // Fetch repository metadata
      const repoUrl = this.getRepositoryFileUrl(url, "repository.json");
      console.log("Loading repository metadata from:", repoUrl);
      const response = await fetch(repoUrl);
      
      if (!response.ok) {
        throw new Error(`Failed to fetch repository metadata: ${response.statusText}`);
      }

      const text = await response.text();
      console.log("Repository metadata content:", text);
      
      if (!text) {
        throw new Error("Empty response from repository");
      }

      const data = JSON.parse(text);
      console.log("Parsed repository metadata:", data);
      const metadata = await repositoryMetadataSchema.parseAsync({
        ...data,
        enabled: true,
        lastSyncedAt: new Date(),
      });

      // Transform asset URLs to absolute URLs
      if (metadata.assets) {
        metadata.assets = {
          icon: metadata.assets.icon ? this.getRepositoryFileUrl(url, metadata.assets.icon) : undefined,
          logo: metadata.assets.logo ? this.getRepositoryFileUrl(url, metadata.assets.logo) : undefined,
          banner: metadata.assets.banner ? this.getRepositoryFileUrl(url, metadata.assets.banner) : undefined,
        };
      }

      // Initialize stats with default values
      metadata.stats = {
        downloads: 0,
        stars: 0,
        plugins: 0,
        ...metadata.stats,
      };

      // Check if repository already exists
      const existing = await db.query.pluginRepositories.findFirst({
        where: eq(pluginRepositories.id, metadata.id),
      });

      if (existing) {
        throw new Error(`Repository ${metadata.id} already exists`);
      }

      // Add to database - only store essential fields
      const storedMetadata: StoredRepositoryMetadata = {
        id: metadata.id,
        url: metadata.url,
        trust: metadata.trust,
        enabled: true,
        lastSyncedAt: new Date(),
      };

      await db.insert(pluginRepositories).values({
        ...storedMetadata,
        createdAt: new Date(),
        updatedAt: new Date(),
      });

      // Add to in-memory cache
      this.repositories.set(metadata.id, metadata);
      this.lastRefresh.set(metadata.id, Date.now());

      // Load plugins
      await this.loadRepositoryPlugins(metadata);

      return metadata;
    } catch (error) {
      console.error("Failed to load repository:", error);
      throw new PluginLoadError(
        `Failed to load repository from ${url}`,
        undefined,
        error
      );
    }
  }

  private transformAssetUrls(plugin: RobotPlugin, baseUrl: string): RobotPlugin {
    const transformUrl = (url: string) => {
      if (url.startsWith('http')) return url;
      return this.getRepositoryFileUrl(baseUrl, url);
    };

    return {
      ...plugin,
      assets: {
        ...plugin.assets,
        thumbnailUrl: transformUrl(plugin.assets.thumbnailUrl),
        images: {
          ...plugin.assets.images,
          main: transformUrl(plugin.assets.images.main),
          angles: plugin.assets.images.angles ? {
            front: plugin.assets.images.angles.front ? transformUrl(plugin.assets.images.angles.front) : undefined,
            side: plugin.assets.images.angles.side ? transformUrl(plugin.assets.images.angles.side) : undefined,
            top: plugin.assets.images.angles.top ? transformUrl(plugin.assets.images.angles.top) : undefined,
          } : undefined,
          dimensions: plugin.assets.images.dimensions ? transformUrl(plugin.assets.images.dimensions) : undefined,
        },
        model: plugin.assets.model ? {
          ...plugin.assets.model,
          url: transformUrl(plugin.assets.model.url),
        } : undefined,
      },
    };
  }

  private async loadRepositoryPlugins(repository: RepositoryMetadata) {
    try {
      // Load plugins index
      const indexUrl = this.getRepositoryFileUrl(repository.url, "plugins/index.json");
      console.log("Loading plugins index from:", indexUrl);
      const indexResponse = await fetch(indexUrl);
      
      if (!indexResponse.ok) {
        throw new Error(`Failed to fetch plugins index (${indexResponse.status})`);
      }

      const indexText = await indexResponse.text();
      console.log("Plugins index content:", indexText);
      
      if (!indexText || indexText.trim() === "") {
        throw new Error("Empty response from plugins index");
      }

      const pluginFiles = JSON.parse(indexText) as string[];
      console.log("Found plugin files:", pluginFiles);

      // Update plugin count in repository stats
      if (repository.stats) {
        repository.stats.plugins = pluginFiles.length;
        // Update in-memory cache only
        this.repositories.set(repository.id, repository);
      }

      // Load each plugin file
      for (const pluginFile of pluginFiles) {
        try {
          const pluginUrl = this.getRepositoryFileUrl(repository.url, `plugins/${pluginFile}`);
          console.log("Loading plugin from:", pluginUrl);
          const pluginResponse = await fetch(pluginUrl);
          
          if (!pluginResponse.ok) {
            console.error(`Failed to load plugin file ${pluginFile}: ${pluginResponse.statusText}`);
            continue;
          }

          const pluginText = await pluginResponse.text();
          if (!pluginText || pluginText.trim() === "") {
            console.error(`Empty response from plugin file ${pluginFile}`);
            continue;
          }

          const pluginData = JSON.parse(pluginText);
          const plugin = await robotPluginSchema.parseAsync(pluginData);
          
          // Transform relative asset URLs to absolute URLs
          const transformedPlugin = this.transformAssetUrls(plugin, repository.url);
          
          // Store the plugin and its repository mapping
          this.plugins.set(transformedPlugin.robotId, transformedPlugin);
          this.pluginToRepo.set(transformedPlugin.robotId, repository.id);
          
          console.log(`Successfully loaded plugin: ${transformedPlugin.name} (${transformedPlugin.robotId})`);
        } catch (error) {
          console.error(`Failed to load plugin ${pluginFile}:`, error);
          // Continue with next plugin if one fails
          continue;
        }
      }
    } catch (error) {
      console.error(`Failed to load plugins for repository ${repository.id}:`, error);
      throw error;
    }
  }

  async removeRepository(id: string): Promise<void> {
    const repository = this.repositories.get(id);
    if (!repository) return;

    if (repository.official) {
      throw new Error("Cannot remove official repository");
    }

    // Remove from database
    await db.delete(pluginRepositories).where(eq(pluginRepositories.id, id));

    // Remove plugins associated with this repository
    for (const [pluginId, repoId] of this.pluginToRepo.entries()) {
      if (repoId === id) {
        this.plugins.delete(pluginId);
        this.pluginToRepo.delete(pluginId);
      }
    }

    // Remove from cache
    this.repositories.delete(id);
    this.lastRefresh.delete(id);
  }

  async loadPluginFromUrl(url: string): Promise<RobotPlugin> {
    try {
      const response = await fetch(url);
      if (!response.ok) {
        throw new Error(`Failed to fetch plugin: ${response.statusText}`);
      }

      const text = await response.text();
      if (!text) {
        throw new Error("Empty response from plugin URL");
      }

      return this.loadPluginFromJson(text);
    } catch (error) {
      throw new PluginLoadError(
        `Failed to load plugin from URL: ${url}`,
        undefined,
        error
      );
    }
  }

  async loadPluginFromJson(jsonString: string): Promise<RobotPlugin> {
    try {
      const data = JSON.parse(jsonString);
      const plugin = await robotPluginSchema.parseAsync(data);
      this.plugins.set(plugin.robotId, plugin);
      return plugin;
    } catch (error) {
      if (error instanceof z.ZodError) {
        throw new PluginLoadError(
          `Invalid plugin format: ${error.errors.map(e => e.message).join(", ")}`,
          undefined,
          error
        );
      }
      throw new PluginLoadError(
        "Failed to load plugin",
        undefined,
        error
      );
    }
  }

  getPlugin(robotId: string): RobotPlugin | undefined {
    return this.plugins.get(robotId);
  }

  getAllPlugins(): RobotPlugin[] {
    return Array.from(this.plugins.values());
  }

  getRepository(id: string): RepositoryMetadata | undefined {
    return this.repositories.get(id);
  }

  getAllRepositories(): RepositoryMetadata[] {
    return Array.from(this.repositories.values());
  }

  registerTransformFunction(name: string, fn: Function): void {
    this.transformFunctions.set(name, fn);
  }

  getTransformFunction(name: string): Function | undefined {
    return this.transformFunctions.get(name);
  }

  private async validatePlugin(data: unknown): Promise<RobotPlugin> {
    return robotPluginSchema.parseAsync(data);
  }

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

  private transformToPoseStamped(params: { x: number; y: number; theta: number }) {
    return {
      header: {
        stamp: {
          sec: Math.floor(Date.now() / 1000),
          nanosec: (Date.now() % 1000) * 1000000
        },
        frame_id: "map"
      },
      pose: {
        position: {
          x: params.x,
          y: params.y,
          z: 0.0
        },
        orientation: {
          x: 0.0,
          y: 0.0,
          z: Math.sin(params.theta / 2),
          w: Math.cos(params.theta / 2)
        }
      }
    };
  }
} 