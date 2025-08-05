ALTER TYPE "public"."step_type" ADD VALUE 'delay' BEFORE 'parallel';--> statement-breakpoint
ALTER TABLE "hs_user" DROP CONSTRAINT "hs_user_active_study_id_hs_study_id_fk";
--> statement-breakpoint
ALTER TABLE "hs_user" DROP COLUMN "active_study_id";