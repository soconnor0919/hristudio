import { db } from "~/server/db";
import { pluginRepositories } from "~/server/db/schema";
import {
  type RobotPlugin,
  type RepositoryMetadata,
  repositoryMetadataSchema,
} from "./types";
import { PluginStore } from "./store";
import { eq } from "drizzle-orm";

// Singleton instance
let store: PluginStore | null = null;

export async function getPluginStore() {
  if (!store) {
    store = new PluginStore();
    try {
      await store.initialize();
    } catch (error) {
      console.error("Failed to initialize plugin store:", error);
      throw error;
    }
  }
  return store;
}

export async function getPlugins(): Promise<RobotPlugin[]> {
  const store = await getPluginStore();
  return store.getAllPlugins();
}

export async function getRepositories(): Promise<RepositoryMetadata[]> {
  const store = await getPluginStore();
  return store.getAllRepositories();
}

export async function addRepository(url: string): Promise<RepositoryMetadata> {
  // Clean URL and ensure it ends with a trailing slash
  const cleanUrl = url.trim().replace(/\/?$/, "/");

  try {
    // Determine if this is a Git URL or repository URL
    const isGitUrl = cleanUrl.includes("github.com/");
    const repoUrl = isGitUrl
      ? cleanUrl
          .replace("github.com/", "")
          .split("/")
          .slice(0, 2)
          .join("/")
          .replace(/\/$/, "")
      : cleanUrl.replace(/\/$/, "");

    // Construct URLs
    const gitUrl = isGitUrl
      ? cleanUrl
      : `https://github.com/${repoUrl.replace("https://", "").replace(".github.io/", "/")}`;
    const repositoryUrl = isGitUrl
      ? `https://${repoUrl.replace(/^[^/]+\//, "").replace(/\/$/, "")}.github.io/${repoUrl.split("/").pop()}`
      : cleanUrl;

    // Fetch repository metadata
    const metadataUrl = `${repositoryUrl}/repository.json`;
    console.log("Loading repository metadata from:", metadataUrl);

    const response = await fetch(metadataUrl);
    if (!response.ok) {
      throw new Error(
        `Failed to fetch repository metadata (${response.status}): ${response.statusText}\n` +
          "Make sure the URL points to a valid plugin repository containing repository.json",
      );
    }

    const text = await response.text();
    if (!text) {
      throw new Error("Empty response from repository");
    }

    console.log("Repository metadata content:", text);
    const metadata = JSON.parse(text);

    // Validate metadata
    const validatedMetadata = await repositoryMetadataSchema.parseAsync({
      ...metadata,
      urls: {
        git: gitUrl,
        repository: repositoryUrl,
      },
      enabled: true,
      lastSyncedAt: new Date(),
    });

    // Check if repository already exists
    const existing = await db.query.pluginRepositories.findFirst({
      where: eq(pluginRepositories.id, validatedMetadata.id),
    });

    if (existing) {
      throw new Error(`Repository ${validatedMetadata.id} already exists`);
    }

    // Add to database
    const [stored] = await db
      .insert(pluginRepositories)
      .values({
        id: validatedMetadata.id,
        urls: {
          git: gitUrl,
          repository: repositoryUrl,
        },
        trust: validatedMetadata.trust,
        enabled: true,
        lastSyncedAt: new Date(),
        createdAt: new Date(),
        updatedAt: new Date(),
      })
      .returning();

    // Clear the store instance to force a fresh load
    store = null;

    return validatedMetadata;
  } catch (error) {
    console.error("Failed to add repository:", error);
    if (error instanceof Error) {
      throw error;
    }
    throw new Error("Failed to add repository");
  }
}

export async function removeRepository(id: string): Promise<void> {
  if (!id) {
    throw new Error("Repository ID is required");
  }

  try {
    // Remove from database first
    await db.delete(pluginRepositories).where(eq(pluginRepositories.id, id));

    // Clear the store instance to force a fresh load
    store = null;
  } catch (error) {
    console.error("Failed to remove repository:", error);
    throw error;
  }
}

export async function getPlugin(
  robotId: string,
): Promise<RobotPlugin | undefined> {
  const store = await getPluginStore();
  return store.getPlugin(robotId);
}
