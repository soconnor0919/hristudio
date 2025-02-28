import { api } from "~/trpc/server";
import { PluginBrowser } from "~/components/store/plugin-browser";
import { PageHeader } from "~/components/layout/page-header";
import { PageContent } from "~/components/layout/page-content";
import { AddRepositoryDialog } from "~/components/store/add-repository-dialog";
import { getCaller } from "~/trpc/server";

export default async function StorePage() {
  // Fetch both plugins and repositories using tRPC
  const caller = await getCaller();
  const [plugins, repositories] = await Promise.all([
    caller.pluginStore.getPlugins(),
    caller.pluginStore.getRepositories(),
  ]);

  return (
    <>
      <PageHeader
        title="Robot Store"
        description="Browse and manage robot plugins"
      >
        <AddRepositoryDialog />
      </PageHeader>
      <PageContent>
        <PluginBrowser 
          repositories={repositories} 
          initialPlugins={plugins} 
        />
      </PageContent>
    </>
  );
} 