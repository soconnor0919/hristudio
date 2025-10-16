import { NextResponse } from "next/server";
import { db } from "~/server/db";
import { trials, experiments, participants } from "~/server/db/schema";
import { eq } from "drizzle-orm";

export async function GET(request: Request) {
  try {
    const { searchParams } = new URL(request.url);
    const trialId = searchParams.get("id");

    if (!trialId) {
      // Get all trials for debugging
      const allTrials = await db
        .select({
          id: trials.id,
          status: trials.status,
          experimentId: trials.experimentId,
          participantId: trials.participantId,
          sessionNumber: trials.sessionNumber,
          scheduledAt: trials.scheduledAt,
          startedAt: trials.startedAt,
        })
        .from(trials)
        .limit(10);

      return NextResponse.json({
        success: true,
        message: "Database connection working",
        trials: allTrials,
        count: allTrials.length,
      });
    }

    // Get specific trial
    const trial = await db
      .select({
        id: trials.id,
        status: trials.status,
        experimentId: trials.experimentId,
        participantId: trials.participantId,
        sessionNumber: trials.sessionNumber,
        scheduledAt: trials.scheduledAt,
        startedAt: trials.startedAt,
        experiment: {
          id: experiments.id,
          name: experiments.name,
        },
        participant: {
          id: participants.id,
          participantCode: participants.participantCode,
        },
      })
      .from(trials)
      .leftJoin(experiments, eq(trials.experimentId, experiments.id))
      .leftJoin(participants, eq(trials.participantId, participants.id))
      .where(eq(trials.id, trialId))
      .limit(1);

    if (!trial[0]) {
      return NextResponse.json({
        success: false,
        error: "Trial not found",
        trialId,
      });
    }

    return NextResponse.json({
      success: true,
      trial: trial[0],
    });
  } catch (error) {
    console.error("Test trial API error:", error);
    return NextResponse.json({
      success: false,
      error: error instanceof Error ? error.message : "Unknown error",
      stack: error instanceof Error ? error.stack : undefined,
    });
  }
}
