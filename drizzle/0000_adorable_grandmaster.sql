CREATE TYPE "public"."activity_type" AS ENUM('study_created', 'study_updated', 'study_deleted', 'ownership_transferred', 'member_added', 'member_removed', 'member_role_changed', 'participant_added', 'participant_updated', 'participant_removed', 'experiment_created', 'experiment_updated', 'experiment_deleted', 'trial_started', 'trial_completed', 'trial_cancelled', 'invitation_sent', 'invitation_accepted', 'invitation_declined', 'invitation_expired', 'invitation_revoked', 'consent_form_added', 'consent_form_signed', 'metadata_updated', 'data_exported');--> statement-breakpoint
CREATE TYPE "public"."experiment_status" AS ENUM('draft', 'active', 'archived');--> statement-breakpoint
CREATE TYPE "public"."invitation_status" AS ENUM('pending', 'accepted', 'declined', 'expired', 'revoked');--> statement-breakpoint
CREATE TYPE "public"."participant_status" AS ENUM('active', 'inactive', 'completed', 'withdrawn');--> statement-breakpoint
CREATE TYPE "public"."study_activity_type" AS ENUM('member_added', 'member_role_changed', 'study_updated', 'participant_added', 'participant_updated', 'invitation_sent', 'invitation_accepted', 'invitation_declined', 'invitation_expired', 'invitation_revoked');--> statement-breakpoint
CREATE TYPE "public"."study_role" AS ENUM('owner', 'admin', 'principal_investigator', 'wizard', 'researcher', 'observer');--> statement-breakpoint
CREATE TABLE "hs_account" (
	"userId" varchar(255) NOT NULL,
	"type" varchar(255) NOT NULL,
	"provider" varchar(255) NOT NULL,
	"providerAccountId" varchar(255) NOT NULL,
	"refresh_token" text,
	"access_token" text,
	"expires_at" integer,
	"token_type" varchar(255),
	"scope" varchar(255),
	"id_token" text,
	"session_state" varchar(255)
);
--> statement-breakpoint
CREATE TABLE "hs_session" (
	"sessionToken" varchar(255) PRIMARY KEY NOT NULL,
	"userId" varchar(255) NOT NULL,
	"expires" timestamp NOT NULL
);
--> statement-breakpoint
CREATE TABLE "hs_user" (
	"id" varchar(255) PRIMARY KEY NOT NULL,
	"email" varchar(255) NOT NULL,
	"first_name" varchar(255),
	"last_name" varchar(255),
	"password" varchar(255),
	"emailVerified" timestamp,
	"image" text
);
--> statement-breakpoint
CREATE TABLE "hs_verificationToken" (
	"identifier" varchar(255) NOT NULL,
	"token" varchar(255) NOT NULL,
	"expires" timestamp NOT NULL
);
--> statement-breakpoint
CREATE TABLE "hs_experiment" (
	"id" integer PRIMARY KEY GENERATED ALWAYS AS IDENTITY (sequence name "hs_experiment_id_seq" INCREMENT BY 1 MINVALUE 1 MAXVALUE 2147483647 START WITH 1 CACHE 1),
	"study_id" integer NOT NULL,
	"title" varchar(256) NOT NULL,
	"description" text,
	"version" integer DEFAULT 1 NOT NULL,
	"status" "experiment_status" DEFAULT 'draft' NOT NULL,
	"steps" jsonb DEFAULT '[]'::jsonb,
	"created_by" varchar(255) NOT NULL,
	"created_at" timestamp with time zone DEFAULT now() NOT NULL,
	"updated_at" timestamp with time zone
);
--> statement-breakpoint
CREATE TABLE "hs_participant" (
	"id" integer PRIMARY KEY GENERATED ALWAYS AS IDENTITY (sequence name "hs_participant_id_seq" INCREMENT BY 1 MINVALUE 1 MAXVALUE 2147483647 START WITH 1 CACHE 1),
	"study_id" integer NOT NULL,
	"identifier" varchar(256),
	"email" varchar(256),
	"first_name" varchar(256),
	"last_name" varchar(256),
	"notes" text,
	"status" "participant_status" DEFAULT 'active' NOT NULL,
	"created_at" timestamp with time zone DEFAULT now() NOT NULL,
	"updated_at" timestamp with time zone
);
--> statement-breakpoint
CREATE TABLE "hs_study" (
	"id" integer PRIMARY KEY GENERATED ALWAYS AS IDENTITY (sequence name "hs_study_id_seq" INCREMENT BY 1 MINVALUE 1 MAXVALUE 2147483647 START WITH 1 CACHE 1),
	"title" varchar(256) NOT NULL,
	"description" text,
	"created_by" varchar(255) NOT NULL,
	"created_at" timestamp with time zone DEFAULT now() NOT NULL,
	"updated_at" timestamp with time zone
);
--> statement-breakpoint
CREATE TABLE "hs_study_activity" (
	"id" integer PRIMARY KEY GENERATED ALWAYS AS IDENTITY (sequence name "hs_study_activity_id_seq" INCREMENT BY 1 MINVALUE 1 MAXVALUE 2147483647 START WITH 1 CACHE 1),
	"study_id" integer NOT NULL,
	"user_id" varchar(255) NOT NULL,
	"type" "activity_type" NOT NULL,
	"description" text NOT NULL,
	"created_at" timestamp with time zone DEFAULT now() NOT NULL
);
--> statement-breakpoint
CREATE TABLE "hs_study_invitation" (
	"id" integer PRIMARY KEY GENERATED ALWAYS AS IDENTITY (sequence name "hs_study_invitation_id_seq" INCREMENT BY 1 MINVALUE 1 MAXVALUE 2147483647 START WITH 1 CACHE 1),
	"study_id" integer NOT NULL,
	"email" varchar(255) NOT NULL,
	"role" "study_role" NOT NULL,
	"token" varchar(255) NOT NULL,
	"status" "invitation_status" DEFAULT 'pending' NOT NULL,
	"expires_at" timestamp with time zone NOT NULL,
	"created_at" timestamp with time zone DEFAULT now() NOT NULL,
	"updated_at" timestamp with time zone,
	"created_by" varchar(255) NOT NULL,
	CONSTRAINT "hs_study_invitation_token_unique" UNIQUE("token")
);
--> statement-breakpoint
CREATE TABLE "hs_study_member" (
	"id" integer PRIMARY KEY GENERATED ALWAYS AS IDENTITY (sequence name "hs_study_member_id_seq" INCREMENT BY 1 MINVALUE 1 MAXVALUE 2147483647 START WITH 1 CACHE 1),
	"study_id" integer NOT NULL,
	"user_id" varchar(255) NOT NULL,
	"role" "study_role" NOT NULL,
	"created_at" timestamp with time zone DEFAULT now() NOT NULL
);
--> statement-breakpoint
CREATE TABLE "hs_study_metadata" (
	"id" integer PRIMARY KEY GENERATED ALWAYS AS IDENTITY (sequence name "hs_study_metadata_id_seq" INCREMENT BY 1 MINVALUE 1 MAXVALUE 2147483647 START WITH 1 CACHE 1),
	"study_id" integer NOT NULL,
	"key" varchar(256) NOT NULL,
	"value" text,
	"created_at" timestamp with time zone DEFAULT now() NOT NULL,
	"updated_at" timestamp with time zone
);
--> statement-breakpoint
CREATE TABLE "hs_permissions" (
	"id" integer PRIMARY KEY GENERATED ALWAYS AS IDENTITY (sequence name "hs_permissions_id_seq" INCREMENT BY 1 MINVALUE 1 MAXVALUE 2147483647 START WITH 1 CACHE 1),
	"code" varchar(50) NOT NULL,
	"name" varchar(100) NOT NULL,
	"description" text,
	"created_at" timestamp DEFAULT now() NOT NULL,
	"updated_at" timestamp DEFAULT now() NOT NULL,
	CONSTRAINT "hs_permissions_code_unique" UNIQUE("code")
);
--> statement-breakpoint
CREATE TABLE "hs_role_permissions" (
	"role_id" integer NOT NULL,
	"permission_id" integer NOT NULL,
	"created_at" timestamp DEFAULT now() NOT NULL,
	CONSTRAINT "hs_role_permissions_role_id_permission_id_pk" PRIMARY KEY("role_id","permission_id")
);
--> statement-breakpoint
CREATE TABLE "hs_roles" (
	"id" integer PRIMARY KEY GENERATED ALWAYS AS IDENTITY (sequence name "hs_roles_id_seq" INCREMENT BY 1 MINVALUE 1 MAXVALUE 2147483647 START WITH 1 CACHE 1),
	"code" varchar(50) NOT NULL,
	"name" varchar(100) NOT NULL,
	"description" text,
	"created_at" timestamp DEFAULT now() NOT NULL,
	"updated_at" timestamp DEFAULT now() NOT NULL,
	CONSTRAINT "hs_roles_code_unique" UNIQUE("code")
);
--> statement-breakpoint
CREATE TABLE "hs_user_roles" (
	"user_id" varchar(255) NOT NULL,
	"role_id" integer NOT NULL,
	"study_id" integer,
	"created_at" timestamp DEFAULT now() NOT NULL,
	CONSTRAINT "hs_user_roles_user_id_role_id_study_id_pk" PRIMARY KEY("user_id","role_id","study_id")
);
--> statement-breakpoint
ALTER TABLE "hs_account" ADD CONSTRAINT "hs_account_userId_hs_user_id_fk" FOREIGN KEY ("userId") REFERENCES "public"."hs_user"("id") ON DELETE no action ON UPDATE no action;--> statement-breakpoint
ALTER TABLE "hs_session" ADD CONSTRAINT "hs_session_userId_hs_user_id_fk" FOREIGN KEY ("userId") REFERENCES "public"."hs_user"("id") ON DELETE no action ON UPDATE no action;--> statement-breakpoint
ALTER TABLE "hs_experiment" ADD CONSTRAINT "hs_experiment_study_id_hs_study_id_fk" FOREIGN KEY ("study_id") REFERENCES "public"."hs_study"("id") ON DELETE cascade ON UPDATE no action;--> statement-breakpoint
ALTER TABLE "hs_experiment" ADD CONSTRAINT "hs_experiment_created_by_hs_user_id_fk" FOREIGN KEY ("created_by") REFERENCES "public"."hs_user"("id") ON DELETE no action ON UPDATE no action;--> statement-breakpoint
ALTER TABLE "hs_participant" ADD CONSTRAINT "hs_participant_study_id_hs_study_id_fk" FOREIGN KEY ("study_id") REFERENCES "public"."hs_study"("id") ON DELETE cascade ON UPDATE no action;--> statement-breakpoint
ALTER TABLE "hs_study" ADD CONSTRAINT "hs_study_created_by_hs_user_id_fk" FOREIGN KEY ("created_by") REFERENCES "public"."hs_user"("id") ON DELETE no action ON UPDATE no action;--> statement-breakpoint
ALTER TABLE "hs_study_activity" ADD CONSTRAINT "hs_study_activity_study_id_hs_study_id_fk" FOREIGN KEY ("study_id") REFERENCES "public"."hs_study"("id") ON DELETE cascade ON UPDATE no action;--> statement-breakpoint
ALTER TABLE "hs_study_activity" ADD CONSTRAINT "hs_study_activity_user_id_hs_user_id_fk" FOREIGN KEY ("user_id") REFERENCES "public"."hs_user"("id") ON DELETE no action ON UPDATE no action;--> statement-breakpoint
ALTER TABLE "hs_study_invitation" ADD CONSTRAINT "hs_study_invitation_study_id_hs_study_id_fk" FOREIGN KEY ("study_id") REFERENCES "public"."hs_study"("id") ON DELETE cascade ON UPDATE no action;--> statement-breakpoint
ALTER TABLE "hs_study_invitation" ADD CONSTRAINT "hs_study_invitation_created_by_hs_user_id_fk" FOREIGN KEY ("created_by") REFERENCES "public"."hs_user"("id") ON DELETE no action ON UPDATE no action;--> statement-breakpoint
ALTER TABLE "hs_study_member" ADD CONSTRAINT "hs_study_member_study_id_hs_study_id_fk" FOREIGN KEY ("study_id") REFERENCES "public"."hs_study"("id") ON DELETE cascade ON UPDATE no action;--> statement-breakpoint
ALTER TABLE "hs_study_member" ADD CONSTRAINT "hs_study_member_user_id_hs_user_id_fk" FOREIGN KEY ("user_id") REFERENCES "public"."hs_user"("id") ON DELETE cascade ON UPDATE no action;--> statement-breakpoint
ALTER TABLE "hs_study_metadata" ADD CONSTRAINT "hs_study_metadata_study_id_hs_study_id_fk" FOREIGN KEY ("study_id") REFERENCES "public"."hs_study"("id") ON DELETE cascade ON UPDATE no action;--> statement-breakpoint
ALTER TABLE "hs_role_permissions" ADD CONSTRAINT "hs_role_permissions_role_id_hs_roles_id_fk" FOREIGN KEY ("role_id") REFERENCES "public"."hs_roles"("id") ON DELETE cascade ON UPDATE no action;--> statement-breakpoint
ALTER TABLE "hs_role_permissions" ADD CONSTRAINT "hs_role_permissions_permission_id_hs_permissions_id_fk" FOREIGN KEY ("permission_id") REFERENCES "public"."hs_permissions"("id") ON DELETE cascade ON UPDATE no action;--> statement-breakpoint
ALTER TABLE "hs_user_roles" ADD CONSTRAINT "hs_user_roles_user_id_hs_user_id_fk" FOREIGN KEY ("user_id") REFERENCES "public"."hs_user"("id") ON DELETE cascade ON UPDATE no action;--> statement-breakpoint
ALTER TABLE "hs_user_roles" ADD CONSTRAINT "hs_user_roles_role_id_hs_roles_id_fk" FOREIGN KEY ("role_id") REFERENCES "public"."hs_roles"("id") ON DELETE cascade ON UPDATE no action;--> statement-breakpoint
ALTER TABLE "hs_user_roles" ADD CONSTRAINT "hs_user_roles_study_id_hs_study_id_fk" FOREIGN KEY ("study_id") REFERENCES "public"."hs_study"("id") ON DELETE cascade ON UPDATE no action;