import { TutorialPage } from "~/components/ui/tutorial-page";
import { Button } from "~/components/ui/button";
import Link from "next/link";

export default function GettingStartedTutorial() {
  return (
    <TutorialPage
      title="Getting Started"
      description="Set up HRIStudio and learn the basics"
      duration="10 min"
      level="Beginner"
      steps={[
        { title: "Clone and install the repository", description: "" },
        { title: "Start the database with Docker", description: "" },
        { title: "Seed the database with sample data", description: "" },
        { title: "Start the development server", description: "" },
        { title: "Log in and explore the interface", description: "" },
      ]}
      nextTutorial={{
        title: "Your First Study",
        href: "/help/tutorials/your-first-study",
      }}
    >
      <h2>Prerequisites</h2>
      <p>Before you begin, make sure you have the following installed:</p>
      <ul>
        <li><strong>Bun</strong> - The package manager for HRIStudio</li>
        <li><strong>Docker</strong> - For running PostgreSQL and MinIO</li>
        <li><strong>Git</strong> - For version control</li>
      </ul>

      <h2>Step 1: Clone the Repository</h2>
      <p>Start by cloning the HRIStudio repository:</p>
      <pre><code>git clone https://github.com/soconnor0919/hristudio.git
cd hristudio</code></pre>

      <h2>Step 2: Install Dependencies</h2>
      <p>HRIStudio uses Bun as its package manager:</p>
      <pre><code>bun install</code></pre>

      <h2>Step 3: Start the Database</h2>
      <p>HRIStudio requires PostgreSQL. The easiest way is using Docker:</p>
      <pre><code># Start PostgreSQL and MinIO (for file storage)
bun run docker:up

# Push database schema
bun db:push

# Seed with sample data
bun db:seed</code></pre>
      <p className="bg-muted p-4 rounded-lg border">
        <strong>Note:</strong> This creates the database schema and populates it with
        sample users, studies, and experiments so you can explore the platform immediately.
      </p>

      <h2>Step 4: Start the Development Server</h2>
      <pre><code>bun dev</code></pre>
      <p>The application will be available at <code>http://localhost:3000</code>.</p>

      <h2>Step 5: Log In</h2>
      <p>Use one of the default accounts:</p>
      <table>
        <thead>
          <tr>
            <th>Role</th>
            <th>Email</th>
            <th>Password</th>
          </tr>
        </thead>
        <tbody>
          <tr>
            <td>Administrator</td>
            <td><code>sean@soconnor.dev</code></td>
            <td><code>password123</code></td>
          </tr>
          <tr>
            <td>Researcher</td>
            <td><code>felipe.perrone@bucknell.edu</code></td>
            <td><code>password123</code></td>
          </tr>
          <tr>
            <td>Wizard</td>
            <td><code>emily.watson@lab.edu</code></td>
            <td><code>password123</code></td>
          </tr>
          <tr>
            <td>Observer</td>
            <td><code>maria.santos@tech.edu</code></td>
            <td><code>password123</code></td>
          </tr>
        </tbody>
      </table>

      <h2>Exploring the Interface</h2>
      <p>After logging in, you&apos;ll see the main dashboard with navigation to:</p>
      <ul>
        <li><strong>Studies</strong> - View and manage your research studies</li>
        <li><strong>Trials</strong> - Monitor and manage experiment trials</li>
        <li><strong>Plugins</strong> - Manage robot integrations</li>
        <li><strong>Admin</strong> - System administration (admins only)</li>
      </ul>

      <h2>Using Simulation Mode</h2>
      <p>If you don&apos;t have a physical robot, enable simulation mode:</p>
      <ol>
        <li>Create or edit <code>hristudio/.env.local</code></li>
        <li>Add: <code>NEXT_PUBLIC_SIMULATION_MODE=true</code></li>
        <li>Restart the dev server</li>
      </ol>
      <p>Simulation mode allows you to test experiments without connecting to a real robot.</p>

      <h2>Troubleshooting</h2>
      
      <h3>Database Connection Failed</h3>
      <pre><code># Check if Docker is running
docker ps

# Restart the database
bun run docker:down
bun run docker:up
bun db:push</code></pre>

      <h3>Port Already in Use</h3>
      <p>If port 3000 is in use:</p>
      <pre><code>PORT=3001 bun dev</code></pre>

      <h3>Seed Script Fails</h3>
      <pre><code># Reset the database
bun run docker:down -v
bun run docker:up
bun db:push
bun db:seed</code></pre>

      <div className="mt-8 flex justify-end">
        <Button asChild>
          <Link href="/help/tutorials/your-first-study">
            Next: Your First Study
          </Link>
        </Button>
      </div>
    </TutorialPage>
  );
}
