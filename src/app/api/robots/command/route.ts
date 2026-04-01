import { NextRequest, NextResponse } from "next/server";
import { auth } from "~/lib/auth";
import { db } from "~/server/db";
import { studyMembers } from "~/server/db/schema";
import { and, eq } from "drizzle-orm";
import { exec } from "child_process";
import { promisify } from "util";

const execAsync = promisify(exec);

export async function POST(request: NextRequest) {
  try {
    const session = await auth.api.getSession({
      headers: request.headers,
    });

    if (!session?.user) {
      return NextResponse.json({ error: "Unauthorized" }, { status: 401 });
    }

    const body = await request.json();
    const { action, studyId, robotId, parameters } = body;

    // Verify user has access to the study
    const membership = await db.query.studyMembers.findFirst({
      where: and(
        eq(studyMembers.studyId, studyId),
        eq(studyMembers.userId, session.user.id),
      ),
    });

    if (!membership || !["owner", "researcher"].includes(membership.role)) {
      return NextResponse.json(
        { error: "Insufficient permissions" },
        { status: 403 },
      );
    }

    const robotIp =
      process.env.NAO_ROBOT_IP || process.env.NAO_IP || "134.82.159.168";
    const password = process.env.NAO_PASSWORD || "robolab";

    switch (action) {
      case "initialize": {
        console.log(`[Robots API] Initializing robot at ${robotIp}`);

        const disableAlCmd = `sshpass -p "${password}" ssh -o StrictHostKeyChecking=no "nao@${robotIp}" "python2 -c \\"import sys; sys.path.append('/opt/aldebaran/lib/python2.7/site-packages'); import naoqi; al = naoqi.ALProxy('ALAutonomousLife', '127.0.0.1', 9559); al.setState('disabled')\\""`;

        const wakeUpCmd = `sshpass -p "${password}" ssh -o StrictHostKeyChecking=no "nao@${robotIp}" "python2 -c \\"import sys; sys.path.append('/opt/aldebaran/lib/python2.7/site-packages'); import naoqi; m = naoqi.ALProxy('ALMotion', '127.0.0.1', 9559); m.wakeUp()\\""`;

        await execAsync(disableAlCmd).catch((e) =>
          console.warn("AL disable failed (non-critical/already disabled):", e),
        );

        await execAsync(wakeUpCmd);

        return NextResponse.json({ success: true });
      }

      case "executeSystemAction": {
        const { id, parameters: actionParams } = parameters ?? {};
        console.log(`[Robots API] Executing system action ${id}`);

        let command = "";

        switch (id) {
          case "say_with_emotion":
          case "say_text_with_emotion": {
            const text = String(actionParams?.text || "Hello");
            const emotion = String(actionParams?.emotion || "happy");
            const tag =
              emotion === "happy"
                ? "^joyful"
                : emotion === "sad"
                  ? "^sad"
                  : emotion === "thinking"
                    ? "^thoughtful"
                    : "^joyful";

            command = `sshpass -p "${password}" ssh -o StrictHostKeyChecking=no "nao@${robotIp}" "python2 -c \\"import sys; sys.path.append('/opt/aldebaran/lib/python2.7/site-packages'); import naoqi; s = naoqi.ALProxy('ALAnimatedSpeech', '127.0.0.1', 9559); s.say('${tag} ${text.replace(/'/g, "\\'")}')\\""`;
            break;
          }

          case "wake_up":
            command = `sshpass -p "${password}" ssh -o StrictHostKeyChecking=no "nao@${robotIp}" "python2 -c \\"import sys; sys.path.append('/opt/aldebaran/lib/python2.7/site-packages'); import naoqi; m = naoqi.ALProxy('ALMotion', '127.0.0.1', 9559); m.wakeUp()\\""`;
            break;

          case "rest":
            command = `sshpass -p "${password}" ssh -o StrictHostKeyChecking=no "nao@${robotIp}" "python2 -c \\"import sys; sys.path.append('/opt/aldebaran/lib/python2.7/site-packages'); import naoqi; m = naoqi.ALProxy('ALMotion', '127.0.0.1', 9559); m.rest()\\""`;
            break;

          case "play_animation_bow":
            command = `sshpass -p "${password}" ssh -o StrictHostKeyChecking=no "nao@${robotIp}" "qicli call ALAnimationPlayer.run 'animations/Stand/Gestures/BowShort_1'"`;
            break;

          case "play_animation_hey":
            command = `sshpass -p "${password}" ssh -o StrictHostKeyChecking=no "nao@${robotIp}" "qicli call ALAnimationPlayer.run 'animations/Stand/Gestures/Hey_1'"`;
            break;

          case "play_animation_show_floor":
            command = `sshpass -p "${password}" ssh -o StrictHostKeyChecking=no "nao@${robotIp}" "qicli call ALAnimationPlayer.run 'animations/Stand/Gestures/ShowFloor_1'"`;
            break;

          case "play_animation_enthusiastic":
            command = `sshpass -p "${password}" ssh -o StrictHostKeyChecking=no "nao@${robotIp}" "qicli call ALAnimationPlayer.run 'animations/Stand/Gestures/Enthusiastic_4'"`;
            break;

          case "play_animation_yes":
            command = `sshpass -p "${password}" ssh -o StrictHostKeyChecking=no "nao@${robotIp}" "qicli call ALAnimationPlayer.run 'animations/Stand/Gestures/Yes_1'"`;
            break;

          case "play_animation_no":
            command = `sshpass -p "${password}" ssh -o StrictHostKeyChecking=no "nao@${robotIp}" "qicli call ALAnimationPlayer.run 'animations/Stand/Gestures/No_3'"`;
            break;

          case "play_animation_idontknow":
            command = `sshpass -p "${password}" ssh -o StrictHostKeyChecking=no "nao@${robotIp}" "qicli call ALAnimationPlayer.run 'animations/Stand/Gestures/IDontKnow_1'"`;
            break;

          default:
            return NextResponse.json(
              { error: `System action ${id} not implemented` },
              { status: 400 },
            );
        }

        await execAsync(command);
        return NextResponse.json({ success: true });
      }

      case "executeSSH": {
        const { command } = parameters ?? {};
        if (!command) {
          return NextResponse.json(
            { error: "Missing command parameter" },
            { status: 400 },
          );
        }

        console.log(`[Robots API] Executing SSH command: ${command}`);

        const sshCmd = `sshpass -p "${password}" ssh -o StrictHostKeyChecking=no -o ConnectTimeout=10 "nao@${robotIp}" "${command}"`;

        try {
          const { stdout, stderr } = await execAsync(sshCmd);
          if (stderr && !stderr.includes("null") && stderr.trim()) {
            console.warn(`[Robots API] SSH stderr: ${stderr}`);
          }
          console.log(`[Robots API] SSH result: ${stdout}`);
          return NextResponse.json({ success: true, stdout, stderr });
        } catch (error) {
          console.error(`[Robots API] SSH command failed:`, error);
          return NextResponse.json(
            { error: error instanceof Error ? error.message : "SSH command failed" },
            { status: 500 },
          );
        }
      }

      default:
        return NextResponse.json(
          { error: `Unknown action: ${action}` },
          { status: 400 },
        );
    }
  } catch (error) {
    console.error("[Robots API] Error:", error);
    return NextResponse.json(
      {
        error: error instanceof Error ? error.message : "Internal server error",
      },
      { status: 500 },
    );
  }
}
