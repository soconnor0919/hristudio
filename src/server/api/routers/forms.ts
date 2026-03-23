import { TRPCError } from "@trpc/server";
import { and, count, desc, eq, ilike, or } from "drizzle-orm";
import { z } from "zod";

import {
  createTRPCRouter,
  protectedProcedure,
  publicProcedure,
} from "~/server/api/trpc";
import {
  activityLogs,
  formResponses,
  formTypeEnum,
  forms,
  formFieldTypeEnum,
  participants,
  studyMembers,
  studies,
  userSystemRoles,
} from "~/server/db/schema";

const formTypes = formTypeEnum.enumValues;
const fieldTypes = formFieldTypeEnum.enumValues;

async function checkStudyAccess(
  db: typeof import("~/server/db").db,
  userId: string,
  studyId: string,
  requiredRole?: string[],
) {
  const adminRole = await db.query.userSystemRoles.findFirst({
    where: and(
      eq(userSystemRoles.userId, userId),
      eq(userSystemRoles.role, "administrator"),
    ),
  });

  if (adminRole) {
    return { role: "administrator", studyId, userId, joinedAt: new Date() };
  }

  const membership = await db.query.studyMembers.findFirst({
    where: and(
      eq(studyMembers.studyId, studyId),
      eq(studyMembers.userId, userId),
    ),
  });

  if (!membership) {
    throw new TRPCError({
      code: "FORBIDDEN",
      message: "You don't have access to this study",
    });
  }

  if (requiredRole && !requiredRole.includes(membership.role)) {
    throw new TRPCError({
      code: "FORBIDDEN",
      message: "You don't have permission to perform this action",
    });
  }

  return membership;
}

export const formsRouter = createTRPCRouter({
  list: protectedProcedure
    .input(
      z.object({
        studyId: z.string().uuid(),
        type: z.enum(formTypes).optional(),
        search: z.string().optional(),
        page: z.number().min(1).default(1),
        limit: z.number().min(1).max(100).default(20),
      }),
    )
    .query(async ({ ctx, input }) => {
      const { studyId, type, search, page, limit } = input;
      const offset = (page - 1) * limit;

      await checkStudyAccess(ctx.db, ctx.session.user.id, studyId);

      const conditions = [eq(forms.studyId, studyId)];
      if (type) {
        conditions.push(eq(forms.type, type));
      }
      if (search) {
        conditions.push(
          or(
            ilike(forms.title, `%${search}%`),
            ilike(forms.description, `%${search}%`),
          )!,
        );
      }

      const [formsList, totalCount] = await Promise.all([
        ctx.db.query.forms.findMany({
          where: and(...conditions),
          with: {
            createdBy: {
              columns: {
                id: true,
                name: true,
                email: true,
              },
            },
          },
          orderBy: [desc(forms.updatedAt)],
          limit,
          offset,
        }),
        ctx.db
          .select({ count: count() })
          .from(forms)
          .where(and(...conditions)),
      ]);

      const formsWithCounts = await Promise.all(
        formsList.map(async (form) => {
          const responseCount = await ctx.db
            .select({ count: count() })
            .from(formResponses)
            .where(eq(formResponses.formId, form.id));
          return {
            ...form,
            _count: { responses: responseCount[0]?.count ?? 0 },
          };
        }),
      );

      return {
        forms: formsWithCounts,
        pagination: {
          page,
          limit,
          total: totalCount[0]?.count ?? 0,
          pages: Math.ceil((totalCount[0]?.count ?? 0) / limit),
        },
      };
    }),

  get: protectedProcedure
    .input(z.object({ id: z.string().uuid() }))
    .query(async ({ ctx, input }) => {
      const form = await ctx.db.query.forms.findFirst({
        where: eq(forms.id, input.id),
        with: {
          createdBy: {
            columns: {
              id: true,
              name: true,
              email: true,
            },
          },
          responses: {
            with: {
              participant: {
                columns: {
                  id: true,
                  participantCode: true,
                  name: true,
                },
              },
            },
            orderBy: [desc(formResponses.submittedAt)],
          },
        },
      });

      if (!form) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "Form not found",
        });
      }

      await checkStudyAccess(ctx.db, ctx.session.user.id, form.studyId);

      return form;
    }),

  create: protectedProcedure
    .input(
      z.object({
        studyId: z.string().uuid(),
        type: z.enum(formTypes),
        title: z.string().min(1).max(255),
        description: z.string().optional(),
        fields: z
          .array(
            z.object({
              id: z.string(),
              type: z.string(),
              label: z.string(),
              required: z.boolean().default(false),
              options: z.array(z.string()).optional(),
              settings: z.record(z.string(), z.any()).optional(),
            }),
          )
          .default([]),
        settings: z.record(z.string(), z.any()).optional(),
        isTemplate: z.boolean().optional(),
        templateName: z.string().max(100).optional(),
      }),
    )
    .mutation(async ({ ctx, input }) => {
      const { isTemplate, templateName, ...formData } = input;

      if (isTemplate && !templateName) {
        throw new TRPCError({
          code: "BAD_REQUEST",
          message: "Template name is required when creating a template",
        });
      }

      await checkStudyAccess(ctx.db, ctx.session.user.id, input.studyId, [
        "owner",
        "researcher",
      ]);

      const [newForm] = await ctx.db
        .insert(forms)
        .values({
          studyId: formData.studyId,
          type: formData.type,
          title: formData.title,
          description: formData.description,
          fields: formData.fields,
          settings: formData.settings ?? {},
          isTemplate: isTemplate ?? false,
          templateName: templateName,
          createdBy: ctx.session.user.id,
        })
        .returning();

      if (!newForm) {
        throw new TRPCError({
          code: "INTERNAL_SERVER_ERROR",
          message: "Failed to create form",
        });
      }

      await ctx.db.insert(activityLogs).values({
        studyId: input.studyId,
        userId: ctx.session.user.id,
        action: "form_created",
        description: `Created form "${newForm.title}"`,
        resourceType: "form",
        resourceId: newForm.id,
      });

      return newForm;
    }),

  update: protectedProcedure
    .input(
      z.object({
        id: z.string().uuid(),
        title: z.string().min(1).max(255).optional(),
        description: z.string().optional(),
        fields: z
          .array(
            z.object({
              id: z.string(),
              type: z.string(),
              label: z.string(),
              required: z.boolean().default(false),
              options: z.array(z.string()).optional(),
              settings: z.record(z.string(), z.any()).optional(),
            }),
          )
          .optional(),
        settings: z.record(z.string(), z.any()).optional(),
      }),
    )
    .mutation(async ({ ctx, input }) => {
      const { id, ...updateData } = input;

      const existingForm = await ctx.db.query.forms.findFirst({
        where: eq(forms.id, id),
      });

      if (!existingForm) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "Form not found",
        });
      }

      await checkStudyAccess(
        ctx.db,
        ctx.session.user.id,
        existingForm.studyId,
        ["owner", "researcher"],
      );

      const [updatedForm] = await ctx.db
        .update(forms)
        .set({
          ...updateData,
          updatedAt: new Date(),
        })
        .where(eq(forms.id, id))
        .returning();

      if (!updatedForm) {
        throw new TRPCError({
          code: "INTERNAL_SERVER_ERROR",
          message: "Failed to update form",
        });
      }

      await ctx.db.insert(activityLogs).values({
        studyId: existingForm.studyId,
        userId: ctx.session.user.id,
        action: "form_updated",
        description: `Updated form "${updatedForm.title}"`,
        resourceType: "form",
        resourceId: id,
      });

      return updatedForm;
    }),

  delete: protectedProcedure
    .input(z.object({ id: z.string().uuid() }))
    .mutation(async ({ ctx, input }) => {
      const form = await ctx.db.query.forms.findFirst({
        where: eq(forms.id, input.id),
      });

      if (!form) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "Form not found",
        });
      }

      await checkStudyAccess(ctx.db, ctx.session.user.id, form.studyId, [
        "owner",
        "researcher",
      ]);

      await ctx.db.delete(forms).where(eq(forms.id, input.id));

      await ctx.db.insert(activityLogs).values({
        studyId: form.studyId,
        userId: ctx.session.user.id,
        action: "form_deleted",
        description: `Deleted form "${form.title}"`,
        resourceType: "form",
        resourceId: input.id,
      });

      return { success: true };
    }),

  setActive: protectedProcedure
    .input(z.object({ id: z.string().uuid() }))
    .mutation(async ({ ctx, input }) => {
      const form = await ctx.db.query.forms.findFirst({
        where: eq(forms.id, input.id),
      });

      if (!form) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "Form not found",
        });
      }

      await checkStudyAccess(ctx.db, ctx.session.user.id, form.studyId, [
        "owner",
        "researcher",
      ]);

      await ctx.db
        .update(forms)
        .set({ active: false })
        .where(eq(forms.studyId, form.studyId));

      const [updatedForm] = await ctx.db
        .update(forms)
        .set({ active: true })
        .where(eq(forms.id, input.id))
        .returning();

      return updatedForm;
    }),

  createVersion: protectedProcedure
    .input(
      z.object({
        id: z.string().uuid(),
        title: z.string().min(1).max(255).optional(),
        description: z.string().optional(),
        fields: z.array(
          z.object({
            id: z.string(),
            type: z.string(),
            label: z.string(),
            required: z.boolean().default(false),
            options: z.array(z.string()).optional(),
            settings: z.record(z.string(), z.any()).optional(),
          }),
        ),
        settings: z.record(z.string(), z.any()).optional(),
      }),
    )
    .mutation(async ({ ctx, input }) => {
      const { id, ...updateData } = input;

      const existingForm = await ctx.db.query.forms.findFirst({
        where: eq(forms.id, id),
      });

      if (!existingForm) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "Form not found",
        });
      }

      await checkStudyAccess(
        ctx.db,
        ctx.session.user.id,
        existingForm.studyId,
        ["owner", "researcher"],
      );

      const latestForm = await ctx.db.query.forms.findFirst({
        where: eq(forms.studyId, existingForm.studyId),
        orderBy: [desc(forms.version)],
      });
      const newVersion = (latestForm?.version ?? 0) + 1;

      const [newForm] = await ctx.db
        .insert(forms)
        .values({
          studyId: existingForm.studyId,
          type: existingForm.type,
          title: updateData.title ?? existingForm.title,
          description: updateData.description ?? existingForm.description,
          fields: updateData.fields ?? existingForm.fields,
          settings: updateData.settings ?? existingForm.settings,
          version: newVersion,
          active: false,
          createdBy: ctx.session.user.id,
        })
        .returning();

      if (!newForm) {
        throw new TRPCError({
          code: "INTERNAL_SERVER_ERROR",
          message: "Failed to create form version",
        });
      }

      await ctx.db.insert(activityLogs).values({
        studyId: existingForm.studyId,
        userId: ctx.session.user.id,
        action: "form_version_created",
        description: `Created version ${newVersion} of form "${newForm.title}"`,
        resourceType: "form",
        resourceId: newForm.id,
      });

      return newForm;
    }),

  getResponses: protectedProcedure
    .input(
      z.object({
        formId: z.string().uuid(),
        page: z.number().min(1).default(1),
        limit: z.number().min(1).max(100).default(20),
        status: z.enum(["pending", "completed", "rejected"]).optional(),
      }),
    )
    .query(async ({ ctx, input }) => {
      const { formId, page, limit, status } = input;
      const offset = (page - 1) * limit;

      const form = await ctx.db.query.forms.findFirst({
        where: eq(forms.id, formId),
      });

      if (!form) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "Form not found",
        });
      }

      await checkStudyAccess(ctx.db, ctx.session.user.id, form.studyId);

      const conditions = [eq(formResponses.formId, formId)];
      if (status) {
        conditions.push(eq(formResponses.status, status));
      }

      const [responses, totalCount] = await Promise.all([
        ctx.db.query.formResponses.findMany({
          where: and(...conditions),
          with: {
            participant: {
              columns: {
                id: true,
                participantCode: true,
                name: true,
                email: true,
              },
            },
          },
          orderBy: [desc(formResponses.submittedAt)],
          limit,
          offset,
        }),
        ctx.db
          .select({ count: count() })
          .from(formResponses)
          .where(and(...conditions)),
      ]);

      return {
        responses,
        pagination: {
          page,
          limit,
          total: totalCount[0]?.count ?? 0,
          pages: Math.ceil((totalCount[0]?.count ?? 0) / limit),
        },
      };
    }),

  exportCsv: protectedProcedure
    .input(z.object({ formId: z.string().uuid() }))
    .query(async ({ ctx, input }) => {
      const form = await ctx.db.query.forms.findFirst({
        where: eq(forms.id, input.formId),
      });

      if (!form) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "Form not found",
        });
      }

      await checkStudyAccess(ctx.db, ctx.session.user.id, form.studyId);

      const responses = await ctx.db.query.formResponses.findMany({
        where: eq(formResponses.formId, input.formId),
        with: {
          participant: {
            columns: {
              id: true,
              participantCode: true,
              name: true,
              email: true,
            },
          },
        },
        orderBy: [desc(formResponses.submittedAt)],
      });

      const fields = form.fields as Array<{
        id: string;
        label: string;
        type: string;
      }>;
      const headers = [
        "Participant Code",
        "Name",
        "Email",
        "Status",
        "Submitted At",
        ...fields.map((f) => f.label),
      ];

      const rows = responses.map((r) => {
        const participantResponses = r.responses as Record<string, any>;
        return [
          r.participant?.participantCode ?? "",
          r.participant?.name ?? "",
          r.participant?.email ?? "",
          r.status,
          r.submittedAt?.toISOString() ?? "",
          ...fields.map((f) => {
            const val = participantResponses[f.id];
            if (val === undefined || val === null) return "";
            if (typeof val === "boolean") return val ? "Yes" : "No";
            return String(val);
          }),
        ];
      });

      const escape = (s: string | null | undefined) =>
        `"${String(s ?? "").replace(/"/g, '""')}"`;
      const csv = [
        headers.map((h) => escape(h)).join(","),
        ...rows.map((row) => row.map((cell) => escape(cell)).join(",")),
      ].join("\n");

      return {
        csv,
        filename: `${form.title.replace(/\s+/g, "_")}_responses.csv`,
      };
    }),

  submitResponse: protectedProcedure
    .input(
      z.object({
        formId: z.string().uuid(),
        participantId: z.string().uuid(),
        responses: z.record(z.string(), z.any()),
        signatureData: z.string().optional(),
      }),
    )
    .mutation(async ({ ctx, input }) => {
      const { formId, participantId, responses, signatureData } = input;

      const form = await ctx.db.query.forms.findFirst({
        where: eq(forms.id, formId),
      });

      if (!form) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "Form not found",
        });
      }

      await checkStudyAccess(ctx.db, ctx.session.user.id, form.studyId);

      const existingResponse = await ctx.db.query.formResponses.findFirst({
        where: and(
          eq(formResponses.formId, formId),
          eq(formResponses.participantId, participantId),
        ),
      });

      if (existingResponse) {
        throw new TRPCError({
          code: "CONFLICT",
          message: "Participant has already submitted this form",
        });
      }

      const [newResponse] = await ctx.db
        .insert(formResponses)
        .values({
          formId,
          participantId,
          responses,
          signatureData,
          status: signatureData ? "completed" : "pending",
          signedAt: signatureData ? new Date() : null,
        })
        .returning();

      return newResponse;
    }),

  listVersions: protectedProcedure
    .input(z.object({ studyId: z.string().uuid() }))
    .query(async ({ ctx, input }) => {
      await checkStudyAccess(ctx.db, ctx.session.user.id, input.studyId);

      const formsList = await ctx.db.query.forms.findMany({
        where: eq(forms.studyId, input.studyId),
        with: {
          createdBy: {
            columns: {
              id: true,
              name: true,
              email: true,
            },
          },
        },
        orderBy: [desc(forms.version)],
      });

      const formsWithCounts = await Promise.all(
        formsList.map(async (form) => {
          const responseCount = await ctx.db
            .select({ count: count() })
            .from(formResponses)
            .where(eq(formResponses.formId, form.id));
          return {
            ...form,
            _count: { responses: responseCount[0]?.count ?? 0 },
          };
        }),
      );

      return formsWithCounts;
    }),

  listTemplates: protectedProcedure.query(async ({ ctx }) => {
    const templates = await ctx.db.query.forms.findMany({
      where: eq(forms.isTemplate, true),
      orderBy: [desc(forms.updatedAt)],
    });

    return templates;
  }),

  createFromTemplate: protectedProcedure
    .input(
      z.object({
        studyId: z.string().uuid(),
        templateId: z.string().uuid(),
        title: z.string().min(1).max(255).optional(),
      }),
    )
    .mutation(async ({ ctx, input }) => {
      await checkStudyAccess(ctx.db, ctx.session.user.id, input.studyId, [
        "owner",
        "researcher",
      ]);

      const template = await ctx.db.query.forms.findFirst({
        where: and(eq(forms.id, input.templateId), eq(forms.isTemplate, true)),
      });

      if (!template) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "Template not found",
        });
      }

      const [newForm] = await ctx.db
        .insert(forms)
        .values({
          studyId: input.studyId,
          type: template.type,
          title: input.title ?? `${template.title} (Copy)`,
          description: template.description,
          fields: template.fields,
          settings: template.settings,
          isTemplate: false,
          createdBy: ctx.session.user.id,
        })
        .returning();

      if (!newForm) {
        throw new TRPCError({
          code: "INTERNAL_SERVER_ERROR",
          message: "Failed to create form from template",
        });
      }

      await ctx.db.insert(activityLogs).values({
        studyId: input.studyId,
        userId: ctx.session.user.id,
        action: "form_created_from_template",
        description: `Created form "${newForm.title}" from template "${template.title}"`,
        resourceType: "form",
        resourceId: newForm.id,
      });

      return newForm;
    }),

  getPublic: publicProcedure
    .input(z.object({ id: z.string().uuid() }))
    .query(async ({ ctx, input }) => {
      const form = await ctx.db.query.forms.findFirst({
        where: and(eq(forms.id, input.id), eq(forms.active, true)),
        columns: {
          id: true,
          studyId: true,
          type: true,
          title: true,
          description: true,
          version: true,
          fields: true,
          settings: true,
        },
      });

      if (!form) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "Form not found or not active",
        });
      }

      const study = await ctx.db.query.studies.findFirst({
        where: eq(studies.id, form.studyId),
        columns: {
          name: true,
        },
      });

      return { ...form, studyName: study?.name };
    }),

  submitPublic: publicProcedure
    .input(
      z.object({
        formId: z.string().uuid(),
        participantCode: z.string().min(1).max(100),
        responses: z.record(z.string(), z.any()),
      }),
    )
    .mutation(async ({ ctx, input }) => {
      const { formId, participantCode, responses } = input;

      const form = await ctx.db.query.forms.findFirst({
        where: and(eq(forms.id, formId), eq(forms.active, true)),
      });

      if (!form) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "Form not found or not active",
        });
      }

      const participant = await ctx.db.query.participants.findFirst({
        where: and(
          eq(participants.studyId, form.studyId),
          eq(participants.participantCode, participantCode),
        ),
      });

      if (!participant) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "Invalid participant code",
        });
      }

      const existingResponse = await ctx.db.query.formResponses.findFirst({
        where: and(
          eq(formResponses.formId, formId),
          eq(formResponses.participantId, participant.id),
        ),
      });

      if (existingResponse) {
        throw new TRPCError({
          code: "CONFLICT",
          message: "You have already submitted this form",
        });
      }

      const [newResponse] = await ctx.db
        .insert(formResponses)
        .values({
          formId,
          participantId: participant.id,
          responses,
          status: "completed",
        })
        .returning();

      return newResponse;
    }),
});
