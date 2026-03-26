import { TutorialPage } from "~/components/ui/tutorial-page";
import { Button } from "~/components/ui/button";
import Link from "next/link";

export default function RobotIntegrationTutorial() {
  return (
    <TutorialPage
      title="Robot Integration"
      description="Connect NAO6 and configure robot plugins"
      duration="20 min"
      level="Advanced"
      steps={[
        { title: "Set up the NAO6 robot", description: "" },
        { title: "Start Docker services", description: "" },
        { title: "Configure HRIStudio", description: "" },
        { title: "Test the connection", description: "" },
        { title: "Troubleshoot common issues", description: "" },
      ]}
      prevTutorial={{
        title: "Wizard Interface",
        href: "/help/tutorials/wizard-interface",
      }}
      nextTutorial={{
        title: "Forms & Surveys",
        href: "/help/tutorials/forms-and-surveys",
      }}
    >
      <h2>Supported Robots</h2>
      <p>HRIStudio supports multiple robot platforms:</p>
      <table>
        <thead>
          <tr><th>Robot</th><th>Protocol</th><th>Capabilities</th></tr>
        </thead>
        <tbody>
          <tr><td>NAO6</td><td>ROS2</td><td>Speech, movement, gestures, sensors</td></tr>
          <tr><td>TurtleBot3</td><td>ROS2</td><td>Navigation, sensors</td></tr>
          <tr><td>Mock Robot</td><td>WebSocket</td><td>All actions (simulation)</td></tr>
        </tbody>
      </table>

      <h2>Step 1: Set Up NAO6 Robot</h2>

      <h3>Network Configuration</h3>
      <ol>
        <li>Connect NAO6 to your network</li>
        <li>Note the robot&apos;s IP address:
          <pre><code># On the robot, say &quot;What is my IP address?&quot;
# Or check robot&apos;s network settings</code></pre>
        </li>
        <li>Verify network access:
          <pre><code>ping nao.local
# Or ping the IP directly:
ping 192.168.1.100</code></pre>
        </li>
      </ol>

      <h3>Wake Up Robot</h3>
      <p>Before connecting, wake up the robot:</p>
      <pre><code>ssh nao@192.168.1.100
# Enter password when prompted

# Wake up the robot
python -c &quot;from naoqi import ALProxy; proxy = ALProxy('ALMotion', '192.168.1.100', 9559); proxy.wakeUp()&quot;</code></pre>

      <h2>Step 2: Start Docker Services</h2>

      <pre><code>cd ~/nao6-hristudio-integration

# Set robot IP
export NAO_IP=192.168.1.100

# Start services
docker compose up -d</code></pre>

      <h3>Services Overview</h3>
      <table>
        <thead>
          <tr><th>Service</th><th>Port</th><th>Purpose</th></tr>
        </thead>
        <tbody>
          <tr><td>nao_driver</td><td>-</td><td>ROS2 driver for NAO</td></tr>
          <tr><td>ros_bridge</td><td>9090</td><td>WebSocket bridge</td></tr>
          <tr><td>ros_api</td><td>-</td><td>Topic introspection</td></tr>
        </tbody>
      </table>

      <h2>Step 3: Configure HRIStudio</h2>

      <h3>Install Robot Plugin</h3>
      <ol>
        <li>Go to <strong>Plugins</strong> in sidebar</li>
        <li>Select your study</li>
        <li>Click <strong>Browse Plugins</strong></li>
        <li>Find <strong>NAO6 Robot (ROS2 Integration)</strong></li>
        <li>Click <strong>Install</strong></li>
      </ol>

      <h3>Configure Plugin</h3>
      <pre><code>Robot Name: NAO6-Lab
Robot IP: 192.168.1.100
WebSocket URL: ws://localhost:9090</code></pre>

      <h3>Environment Variables</h3>
      <p>Create <code>hristudio/.env.local</code>:</p>
      <pre><code># Robot connection
NAO_ROBOT_IP=192.168.1.100
NAO_PASSWORD=robolab
NAO_USERNAME=nao

# WebSocket bridge
NEXT_PUBLIC_ROS_BRIDGE_URL=ws://localhost:9090</code></pre>

      <h2>Step 4: Test Connection</h2>
      <ol>
        <li>Navigate to: <code>http://localhost:3000/nao-test</code></li>
        <li>Click <strong>Connect</strong></li>
        <li>Verify connection status shows &quot;Connected&quot;</li>
        <li>Test basic actions (Say, Wave, Move)</li>
      </ol>

      <h2>Robot Actions Reference</h2>

      <h3>Speech Actions</h3>
      <table>
        <thead>
          <tr><th>Action</th><th>Parameters</th><th>Description</th></tr>
        </thead>
        <tbody>
          <tr><td>say_text</td><td>text</td><td>Speak text</td></tr>
          <tr><td>say_with_emotion</td><td>text, emotion</td><td>Emotional speech</td></tr>
          <tr><td>set_volume</td><td>level</td><td>Set speech volume</td></tr>
        </tbody>
      </table>

      <h3>Movement Actions</h3>
      <table>
        <thead>
          <tr><th>Action</th><th>Parameters</th><th>Description</th></tr>
        </thead>
        <tbody>
          <tr><td>walk_forward</td><td>speed, duration</td><td>Walk forward</td></tr>
          <tr><td>walk_backward</td><td>speed</td><td>Walk backward</td></tr>
          <tr><td>turn_left</td><td>speed</td><td>Turn left</td></tr>
          <tr><td>turn_right</td><td>speed</td><td>Turn right</td></tr>
        </tbody>
      </table>

      <h2>Troubleshooting</h2>

      <h3>Robot Not Found</h3>
      <pre><code>Error: Cannot connect to robot at 192.168.1.100

Solutions:
1. Verify IP address: ping 192.168.1.100
2. Check robot is powered on
3. Verify network connectivity
4. Try nao.local hostname</code></pre>

      <h3>WebSocket Connection Failed</h3>
      <pre><code>Error: WebSocket connection to ws://localhost:9090 failed

Solutions:
1. Check Docker is running: docker ps
2. Verify ros_bridge container
3. Check port 9090 is not blocked
4. Restart services: docker compose restart</code></pre>

      <div className="mt-8 flex justify-between">
        <Button variant="outline" asChild>
          <Link href="/help/tutorials/wizard-interface">
            Previous: Wizard Interface
          </Link>
        </Button>
        <Button asChild>
          <Link href="/help/tutorials/forms-and-surveys">
            Next: Forms & Surveys
          </Link>
        </Button>
      </div>
    </TutorialPage>
  );
}
