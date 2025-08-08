import { and, count, desc, eq, gte, inArray, sql } from "drizzle-orm";
import { z } from "zod";

import { createTRPCRouter, protectedProcedure } from "~/server/api/trpc";
import {
  activityLogs,
  experiments,
  participants,
  studies,
  studyMembers,
  trials,
  users,
  userSystemRoles,
} from "~/server/db/schema";

export const dashboardRouter = createTRPCRouter({
  getRecentActivity: protectedProcedure
    .input(
      z.object({
        limit: z.number().min(1).max(20).default(10),
        studyId: z.string().uuid().optional(),
      }),
    )
    .query(async ({ ctx, input }) => {
      const userId = ctx.session.user.id;

      // Get studies the user has access to
      const accessibleStudies = await ctx.db
        .select({ studyId: studyMembers.studyId })
        .from(studyMembers)
        .where(eq(studyMembers.userId, userId));

      const studyIds = accessibleStudies.map((s) => s.studyId);

      // If no accessible studies, return empty
      if (studyIds.length === 0) {
        return [];
      }

      // Build where conditions
      const whereConditions = input.studyId
        ? eq(activityLogs.studyId, input.studyId)
        : inArray(activityLogs.studyId, studyIds);

      // Get recent activity logs
      const activities = await ctx.db
        .select({
          id: activityLogs.id,
          action: activityLogs.action,
          description: activityLogs.description,
          createdAt: activityLogs.createdAt,
          user: {
            name: users.name,
            email: users.email,
          },
          study: {
            name: studies.name,
          },
        })
        .from(activityLogs)
        .innerJoin(users, eq(activityLogs.userId, users.id))
        .innerJoin(studies, eq(activityLogs.studyId, studies.id))
        .where(whereConditions)
        .orderBy(desc(activityLogs.createdAt))
        .limit(input.limit);

      return activities.map((activity) => ({
        id: activity.id,
        type: activity.action,
        title: activity.description,
        description: `${activity.study.name} - ${activity.user.name}`,
        time: activity.createdAt,
        status: "info" as const,
      }));
    }),

  getStudyProgress: protectedProcedure
    .input(
      z.object({
        limit: z.number().min(1).max(10).default(5),
      }),
    )
    .query(async ({ ctx, input }) => {
      const userId = ctx.session.user.id;

      // Get studies the user has access to with participant counts
      const studyProgress = await ctx.db
        .select({
          id: studies.id,
          name: studies.name,
          status: studies.status,
          createdAt: studies.createdAt,
          totalParticipants: count(participants.id),
        })
        .from(studies)
        .innerJoin(studyMembers, eq(studies.id, studyMembers.studyId))
        .leftJoin(participants, eq(studies.id, participants.studyId))
        .where(
          and(eq(studyMembers.userId, userId), eq(studies.status, "active")),
        )
        .groupBy(studies.id, studies.name, studies.status, studies.createdAt)
        .orderBy(desc(studies.createdAt))
        .limit(input.limit);

      // Get trial completion counts for each study
      const studyIds = studyProgress.map((s) => s.id);

      const trialCounts =
        studyIds.length > 0
          ? await ctx.db
              .select({
                studyId: experiments.studyId,
                completedTrials: count(trials.id),
              })
              .from(experiments)
              .innerJoin(trials, eq(experiments.id, trials.experimentId))
              .where(
                and(
                  inArray(experiments.studyId, studyIds),
                  eq(trials.status, "completed"),
                ),
              )
              .groupBy(experiments.studyId)
          : [];

      const trialCountMap = new Map(
        trialCounts.map((tc) => [tc.studyId, tc.completedTrials]),
      );

      return studyProgress.map((study) => {
        const completedTrials = trialCountMap.get(study.id) ?? 0;
        const totalParticipants = study.totalParticipants;

        // Calculate progress based on completed trials vs participants
        // If no participants, progress is 0; if trials >= participants, progress is 100%
        const progress =
          totalParticipants > 0
            ? Math.min(
                100,
                Math.round((completedTrials / totalParticipants) * 100),
              )
            : 0;

        return {
          id: study.id,
          name: study.name,
          progress,
          participants: completedTrials, // Using completed trials as active participants
          totalParticipants,
          status: study.status,
        };
      });
    }),

  getStats: protectedProcedure.query(async ({ ctx }) => {
    const userId = ctx.session.user.id;

    // Get studies the user has access to
    const accessibleStudies = await ctx.db
      .select({ studyId: studyMembers.studyId })
      .from(studyMembers)
      .where(eq(studyMembers.userId, userId));

    const studyIds = accessibleStudies.map((s) => s.studyId);

    if (studyIds.length === 0) {
      return {
        totalStudies: 0,
        totalExperiments: 0,
        totalParticipants: 0,
        totalTrials: 0,
        activeTrials: 0,
        scheduledTrials: 0,
        completedToday: 0,
      };
    }

    // Get total counts
    const [studyCount] = await ctx.db
      .select({ count: count() })
      .from(studies)
      .where(inArray(studies.id, studyIds));

    const [experimentCount] = await ctx.db
      .select({ count: count() })
      .from(experiments)
      .where(inArray(experiments.studyId, studyIds));

    const [participantCount] = await ctx.db
      .select({ count: count() })
      .from(participants)
      .where(inArray(participants.studyId, studyIds));

    const [trialCount] = await ctx.db
      .select({ count: count() })
      .from(trials)
      .innerJoin(experiments, eq(trials.experimentId, experiments.id))
      .where(inArray(experiments.studyId, studyIds));

    // Get active trials count
    const [activeTrialsCount] = await ctx.db
      .select({ count: count() })
      .from(trials)
      .innerJoin(experiments, eq(trials.experimentId, experiments.id))
      .where(
        and(
          inArray(experiments.studyId, studyIds),
          eq(trials.status, "in_progress"),
        ),
      );

    // Get scheduled trials count
    const [scheduledTrialsCount] = await ctx.db
      .select({ count: count() })
      .from(trials)
      .innerJoin(experiments, eq(trials.experimentId, experiments.id))
      .where(
        and(
          inArray(experiments.studyId, studyIds),
          eq(trials.status, "scheduled"),
        ),
      );

    // Get today's completed trials
    const today = new Date();
    today.setHours(0, 0, 0, 0);

    const [completedTodayCount] = await ctx.db
      .select({ count: count() })
      .from(trials)
      .innerJoin(experiments, eq(trials.experimentId, experiments.id))
      .where(
        and(
          inArray(experiments.studyId, studyIds),
          eq(trials.status, "completed"),
          gte(trials.completedAt, today),
        ),
      );

    return {
      totalStudies: studyCount?.count ?? 0,
      totalExperiments: experimentCount?.count ?? 0,
      totalParticipants: participantCount?.count ?? 0,
      totalTrials: trialCount?.count ?? 0,
      activeTrials: activeTrialsCount?.count ?? 0,
      scheduledTrials: scheduledTrialsCount?.count ?? 0,
      completedToday: completedTodayCount?.count ?? 0,
    };
  }),

  debug: protectedProcedure.query(async ({ ctx }) => {
    const userId = ctx.session.user.id;

    // Get user info
    const user = await ctx.db.query.users.findFirst({
      where: eq(users.id, userId),
    });

    // Get user system roles
    const systemRoles = await ctx.db.query.userSystemRoles.findMany({
      where: eq(userSystemRoles.userId, userId),
    });

    // Get user study memberships
    const studyMemberships = await ctx.db.query.studyMembers.findMany({
      where: eq(studyMembers.userId, userId),
      with: {
        study: {
          columns: {
            id: true,
            name: true,
            status: true,
          },
        },
      },
    });

    // Get all studies (admin view)
    const allStudies = await ctx.db.query.studies.findMany({
      columns: {
        id: true,
        name: true,
        status: true,
        createdBy: true,
      },
      where: sql`deleted_at IS NULL`,
      limit: 10,
    });

    return {
      user: user
        ? {
            id: user.id,
            email: user.email,
            name: user.name,
          }
        : null,
      systemRoles: systemRoles.map((r) => r.role),
      studyMemberships: studyMemberships.map((m) => ({
        studyId: m.studyId,
        role: m.role,
        study: m.study,
      })),
      allStudies,
      session: {
        userId: ctx.session.user.id,
        userEmail: ctx.session.user.email,
        userRole: ctx.session.user.roles?.[0]?.role ?? null,
      },
    };
  }),
});
