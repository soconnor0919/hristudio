CREATE TYPE "public"."communication_protocol" AS ENUM('rest', 'ros2', 'custom');--> statement-breakpoint
CREATE TYPE "public"."experiment_status" AS ENUM('draft', 'testing', 'ready', 'deprecated');--> statement-breakpoint
CREATE TYPE "public"."export_status" AS ENUM('pending', 'processing', 'completed', 'failed');--> statement-breakpoint
CREATE TYPE "public"."media_type" AS ENUM('video', 'audio', 'image');--> statement-breakpoint
CREATE TYPE "public"."plugin_status" AS ENUM('active', 'deprecated', 'disabled');--> statement-breakpoint
CREATE TYPE "public"."step_type" AS ENUM('wizard', 'robot', 'parallel', 'conditional');--> statement-breakpoint
CREATE TYPE "public"."study_member_role" AS ENUM('owner', 'researcher', 'wizard', 'observer');--> statement-breakpoint
CREATE TYPE "public"."study_status" AS ENUM('draft', 'active', 'completed', 'archived');--> statement-breakpoint
CREATE TYPE "public"."system_role" AS ENUM('administrator', 'researcher', 'wizard', 'observer');--> statement-breakpoint
CREATE TYPE "public"."trial_status" AS ENUM('scheduled', 'in_progress', 'completed', 'aborted', 'failed');--> statement-breakpoint
CREATE TYPE "public"."trust_level" AS ENUM('official', 'verified', 'community');--> statement-breakpoint
CREATE TABLE "hs_account" (
	"user_id" uuid NOT NULL,
	"type" varchar(255) NOT NULL,
	"provider" varchar(255) NOT NULL,
	"provider_account_id" varchar(255) NOT NULL,
	"refresh_token" text,
	"access_token" text,
	"expires_at" integer,
	"token_type" varchar(255),
	"scope" varchar(255),
	"id_token" text,
	"session_state" varchar(255),
	"created_at" timestamp with time zone DEFAULT CURRENT_TIMESTAMP NOT NULL,
	"updated_at" timestamp with time zone DEFAULT CURRENT_TIMESTAMP NOT NULL,
	CONSTRAINT "hs_account_provider_provider_account_id_pk" PRIMARY KEY("provider","provider_account_id")
);
--> statement-breakpoint
CREATE TABLE "hs_action" (
	"id" uuid PRIMARY KEY DEFAULT gen_random_uuid() NOT NULL,
	"step_id" uuid NOT NULL,
	"name" varchar(255) NOT NULL,
	"description" text,
	"type" varchar(100) NOT NULL,
	"order_index" integer NOT NULL,
	"parameters" jsonb DEFAULT '{}'::jsonb,
	"validation_schema" jsonb,
	"timeout" integer,
	"retry_count" integer DEFAULT 0 NOT NULL,
	"created_at" timestamp with time zone DEFAULT CURRENT_TIMESTAMP NOT NULL,
	"updated_at" timestamp with time zone DEFAULT CURRENT_TIMESTAMP NOT NULL,
	CONSTRAINT "hs_action_step_id_order_index_unique" UNIQUE("step_id","order_index")
);
--> statement-breakpoint
CREATE TABLE "hs_activity_log" (
	"id" uuid PRIMARY KEY DEFAULT gen_random_uuid() NOT NULL,
	"study_id" uuid,
	"user_id" uuid,
	"action" varchar(100) NOT NULL,
	"resource_type" varchar(50),
	"resource_id" uuid,
	"description" text,
	"ip_address" "inet",
	"user_agent" text,
	"metadata" jsonb DEFAULT '{}'::jsonb,
	"created_at" timestamp with time zone DEFAULT CURRENT_TIMESTAMP NOT NULL
);
--> statement-breakpoint
CREATE TABLE "hs_annotation" (
	"id" uuid PRIMARY KEY DEFAULT gen_random_uuid() NOT NULL,
	"trial_id" uuid NOT NULL,
	"annotator_id" uuid NOT NULL,
	"timestamp_start" timestamp with time zone NOT NULL,
	"timestamp_end" timestamp with time zone,
	"category" varchar(100),
	"label" varchar(100),
	"description" text,
	"tags" jsonb DEFAULT '[]'::jsonb,
	"metadata" jsonb DEFAULT '{}'::jsonb,
	"created_at" timestamp with time zone DEFAULT CURRENT_TIMESTAMP NOT NULL,
	"updated_at" timestamp with time zone DEFAULT CURRENT_TIMESTAMP NOT NULL
);
--> statement-breakpoint
CREATE TABLE "hs_attachment" (
	"id" uuid PRIMARY KEY DEFAULT gen_random_uuid() NOT NULL,
	"resource_type" varchar(50) NOT NULL,
	"resource_id" uuid NOT NULL,
	"file_name" varchar(255) NOT NULL,
	"file_size" bigint NOT NULL,
	"file_path" text NOT NULL,
	"content_type" varchar(100),
	"description" text,
	"uploaded_by" uuid NOT NULL,
	"created_at" timestamp with time zone DEFAULT CURRENT_TIMESTAMP NOT NULL
);
--> statement-breakpoint
CREATE TABLE "hs_audit_log" (
	"id" uuid PRIMARY KEY DEFAULT gen_random_uuid() NOT NULL,
	"user_id" uuid,
	"action" varchar(100) NOT NULL,
	"resource_type" varchar(50),
	"resource_id" uuid,
	"changes" jsonb DEFAULT '{}'::jsonb,
	"ip_address" "inet",
	"user_agent" text,
	"created_at" timestamp with time zone DEFAULT CURRENT_TIMESTAMP NOT NULL
);
--> statement-breakpoint
CREATE TABLE "hs_comment" (
	"id" uuid PRIMARY KEY DEFAULT gen_random_uuid() NOT NULL,
	"parent_id" uuid,
	"resource_type" varchar(50) NOT NULL,
	"resource_id" uuid NOT NULL,
	"author_id" uuid NOT NULL,
	"content" text NOT NULL,
	"metadata" jsonb,
	"created_at" timestamp with time zone DEFAULT CURRENT_TIMESTAMP NOT NULL,
	"updated_at" timestamp with time zone DEFAULT CURRENT_TIMESTAMP NOT NULL
);
--> statement-breakpoint
CREATE TABLE "hs_consent_form" (
	"id" uuid PRIMARY KEY DEFAULT gen_random_uuid() NOT NULL,
	"study_id" uuid NOT NULL,
	"version" integer DEFAULT 1 NOT NULL,
	"title" varchar(255) NOT NULL,
	"content" text NOT NULL,
	"active" boolean DEFAULT true NOT NULL,
	"created_by" uuid NOT NULL,
	"created_at" timestamp with time zone DEFAULT CURRENT_TIMESTAMP NOT NULL,
	"storage_path" text,
	CONSTRAINT "hs_consent_form_study_id_version_unique" UNIQUE("study_id","version")
);
--> statement-breakpoint
CREATE TABLE "hs_experiment" (
	"id" uuid PRIMARY KEY DEFAULT gen_random_uuid() NOT NULL,
	"study_id" uuid NOT NULL,
	"name" varchar(255) NOT NULL,
	"description" text,
	"version" integer DEFAULT 1 NOT NULL,
	"robot_id" uuid,
	"status" "experiment_status" DEFAULT 'draft' NOT NULL,
	"estimated_duration" integer,
	"created_by" uuid NOT NULL,
	"created_at" timestamp with time zone DEFAULT CURRENT_TIMESTAMP NOT NULL,
	"updated_at" timestamp with time zone DEFAULT CURRENT_TIMESTAMP NOT NULL,
	"metadata" jsonb DEFAULT '{}'::jsonb,
	"deleted_at" timestamp with time zone,
	CONSTRAINT "hs_experiment_study_id_name_version_unique" UNIQUE("study_id","name","version")
);
--> statement-breakpoint
CREATE TABLE "hs_export_job" (
	"id" uuid PRIMARY KEY DEFAULT gen_random_uuid() NOT NULL,
	"study_id" uuid NOT NULL,
	"requested_by" uuid NOT NULL,
	"export_type" varchar(50) NOT NULL,
	"format" varchar(20) NOT NULL,
	"filters" jsonb DEFAULT '{}'::jsonb,
	"status" "export_status" DEFAULT 'pending' NOT NULL,
	"storage_path" text,
	"expires_at" timestamp with time zone,
	"created_at" timestamp with time zone DEFAULT CURRENT_TIMESTAMP NOT NULL,
	"completed_at" timestamp with time zone,
	"error_message" text
);
--> statement-breakpoint
CREATE TABLE "hs_media_capture" (
	"id" uuid PRIMARY KEY DEFAULT gen_random_uuid() NOT NULL,
	"trial_id" uuid NOT NULL,
	"media_type" "media_type",
	"storage_path" text NOT NULL,
	"file_size" bigint,
	"duration" integer,
	"format" varchar(20),
	"resolution" varchar(20),
	"start_timestamp" timestamp with time zone,
	"end_timestamp" timestamp with time zone,
	"metadata" jsonb DEFAULT '{}'::jsonb,
	"created_at" timestamp with time zone DEFAULT CURRENT_TIMESTAMP NOT NULL
);
--> statement-breakpoint
CREATE TABLE "hs_participant_consent" (
	"id" uuid PRIMARY KEY DEFAULT gen_random_uuid() NOT NULL,
	"participant_id" uuid NOT NULL,
	"consent_form_id" uuid NOT NULL,
	"signed_at" timestamp with time zone DEFAULT CURRENT_TIMESTAMP NOT NULL,
	"signature_data" text,
	"ip_address" "inet",
	"storage_path" text,
	CONSTRAINT "hs_participant_consent_participant_id_consent_form_id_unique" UNIQUE("participant_id","consent_form_id")
);
--> statement-breakpoint
CREATE TABLE "hs_participant" (
	"id" uuid PRIMARY KEY DEFAULT gen_random_uuid() NOT NULL,
	"study_id" uuid NOT NULL,
	"participant_code" varchar(50) NOT NULL,
	"email" varchar(255),
	"name" varchar(255),
	"demographics" jsonb DEFAULT '{}'::jsonb,
	"consent_given" boolean DEFAULT false NOT NULL,
	"consent_date" timestamp with time zone,
	"notes" text,
	"created_at" timestamp with time zone DEFAULT CURRENT_TIMESTAMP NOT NULL,
	"updated_at" timestamp with time zone DEFAULT CURRENT_TIMESTAMP NOT NULL,
	CONSTRAINT "hs_participant_study_id_participant_code_unique" UNIQUE("study_id","participant_code")
);
--> statement-breakpoint
CREATE TABLE "hs_permission" (
	"id" uuid PRIMARY KEY DEFAULT gen_random_uuid() NOT NULL,
	"name" varchar(100) NOT NULL,
	"description" text,
	"resource" varchar(50) NOT NULL,
	"action" varchar(50) NOT NULL,
	"created_at" timestamp with time zone DEFAULT CURRENT_TIMESTAMP NOT NULL,
	CONSTRAINT "hs_permission_name_unique" UNIQUE("name")
);
--> statement-breakpoint
CREATE TABLE "hs_plugin" (
	"id" uuid PRIMARY KEY DEFAULT gen_random_uuid() NOT NULL,
	"robot_id" uuid,
	"name" varchar(255) NOT NULL,
	"version" varchar(50) NOT NULL,
	"description" text,
	"author" varchar(255),
	"repository_url" text,
	"trust_level" "trust_level",
	"status" "plugin_status" DEFAULT 'active' NOT NULL,
	"configuration_schema" jsonb,
	"action_definitions" jsonb DEFAULT '[]'::jsonb,
	"created_at" timestamp with time zone DEFAULT CURRENT_TIMESTAMP NOT NULL,
	"updated_at" timestamp with time zone DEFAULT CURRENT_TIMESTAMP NOT NULL,
	"metadata" jsonb DEFAULT '{}'::jsonb,
	CONSTRAINT "hs_plugin_name_version_unique" UNIQUE("name","version")
);
--> statement-breakpoint
CREATE TABLE "hs_robot" (
	"id" uuid PRIMARY KEY DEFAULT gen_random_uuid() NOT NULL,
	"name" varchar(255) NOT NULL,
	"manufacturer" varchar(255),
	"model" varchar(255),
	"description" text,
	"capabilities" jsonb DEFAULT '[]'::jsonb,
	"communication_protocol" "communication_protocol",
	"created_at" timestamp with time zone DEFAULT CURRENT_TIMESTAMP NOT NULL,
	"updated_at" timestamp with time zone DEFAULT CURRENT_TIMESTAMP NOT NULL
);
--> statement-breakpoint
CREATE TABLE "hs_role_permission" (
	"id" uuid PRIMARY KEY DEFAULT gen_random_uuid() NOT NULL,
	"role" "system_role" NOT NULL,
	"permission_id" uuid NOT NULL,
	CONSTRAINT "hs_role_permission_role_permission_id_unique" UNIQUE("role","permission_id")
);
--> statement-breakpoint
CREATE TABLE "hs_sensor_data" (
	"id" uuid PRIMARY KEY DEFAULT gen_random_uuid() NOT NULL,
	"trial_id" uuid NOT NULL,
	"sensor_type" varchar(50) NOT NULL,
	"timestamp" timestamp with time zone NOT NULL,
	"data" jsonb NOT NULL,
	"robot_state" jsonb DEFAULT '{}'::jsonb,
	"created_at" timestamp with time zone DEFAULT CURRENT_TIMESTAMP NOT NULL
);
--> statement-breakpoint
CREATE TABLE "hs_session" (
	"id" uuid PRIMARY KEY DEFAULT gen_random_uuid() NOT NULL,
	"session_token" varchar(255) NOT NULL,
	"user_id" uuid NOT NULL,
	"expires" timestamp with time zone NOT NULL,
	"created_at" timestamp with time zone DEFAULT CURRENT_TIMESTAMP NOT NULL,
	"updated_at" timestamp with time zone DEFAULT CURRENT_TIMESTAMP NOT NULL,
	CONSTRAINT "hs_session_session_token_unique" UNIQUE("session_token")
);
--> statement-breakpoint
CREATE TABLE "hs_shared_resource" (
	"id" uuid PRIMARY KEY DEFAULT gen_random_uuid() NOT NULL,
	"study_id" uuid NOT NULL,
	"resource_type" varchar(50) NOT NULL,
	"resource_id" uuid NOT NULL,
	"shared_by" uuid NOT NULL,
	"share_token" varchar(255),
	"permissions" jsonb DEFAULT '["read"]'::jsonb,
	"expires_at" timestamp with time zone,
	"access_count" integer DEFAULT 0 NOT NULL,
	"created_at" timestamp with time zone DEFAULT CURRENT_TIMESTAMP NOT NULL,
	CONSTRAINT "hs_shared_resource_share_token_unique" UNIQUE("share_token")
);
--> statement-breakpoint
CREATE TABLE "hs_step" (
	"id" uuid PRIMARY KEY DEFAULT gen_random_uuid() NOT NULL,
	"experiment_id" uuid NOT NULL,
	"name" varchar(255) NOT NULL,
	"description" text,
	"type" "step_type" NOT NULL,
	"order_index" integer NOT NULL,
	"duration_estimate" integer,
	"required" boolean DEFAULT true NOT NULL,
	"conditions" jsonb DEFAULT '{}'::jsonb,
	"created_at" timestamp with time zone DEFAULT CURRENT_TIMESTAMP NOT NULL,
	"updated_at" timestamp with time zone DEFAULT CURRENT_TIMESTAMP NOT NULL,
	CONSTRAINT "hs_step_experiment_id_order_index_unique" UNIQUE("experiment_id","order_index")
);
--> statement-breakpoint
CREATE TABLE "hs_study" (
	"id" uuid PRIMARY KEY DEFAULT gen_random_uuid() NOT NULL,
	"name" varchar(255) NOT NULL,
	"description" text,
	"institution" varchar(255),
	"irb_protocol" varchar(100),
	"status" "study_status" DEFAULT 'draft' NOT NULL,
	"created_by" uuid NOT NULL,
	"created_at" timestamp with time zone DEFAULT CURRENT_TIMESTAMP NOT NULL,
	"updated_at" timestamp with time zone DEFAULT CURRENT_TIMESTAMP NOT NULL,
	"metadata" jsonb DEFAULT '{}'::jsonb,
	"settings" jsonb DEFAULT '{}'::jsonb,
	"deleted_at" timestamp with time zone
);
--> statement-breakpoint
CREATE TABLE "hs_study_member" (
	"id" uuid PRIMARY KEY DEFAULT gen_random_uuid() NOT NULL,
	"study_id" uuid NOT NULL,
	"user_id" uuid NOT NULL,
	"role" "study_member_role" NOT NULL,
	"permissions" jsonb DEFAULT '[]'::jsonb,
	"joined_at" timestamp with time zone DEFAULT CURRENT_TIMESTAMP NOT NULL,
	"invited_by" uuid,
	CONSTRAINT "hs_study_member_study_id_user_id_unique" UNIQUE("study_id","user_id")
);
--> statement-breakpoint
CREATE TABLE "hs_study_plugin" (
	"id" uuid PRIMARY KEY DEFAULT gen_random_uuid() NOT NULL,
	"study_id" uuid NOT NULL,
	"plugin_id" uuid NOT NULL,
	"configuration" jsonb DEFAULT '{}'::jsonb,
	"installed_at" timestamp with time zone DEFAULT CURRENT_TIMESTAMP NOT NULL,
	"installed_by" uuid NOT NULL,
	CONSTRAINT "hs_study_plugin_study_id_plugin_id_unique" UNIQUE("study_id","plugin_id")
);
--> statement-breakpoint
CREATE TABLE "hs_system_setting" (
	"id" uuid PRIMARY KEY DEFAULT gen_random_uuid() NOT NULL,
	"key" varchar(100) NOT NULL,
	"value" jsonb NOT NULL,
	"description" text,
	"updated_by" uuid,
	"updated_at" timestamp with time zone DEFAULT CURRENT_TIMESTAMP NOT NULL,
	CONSTRAINT "hs_system_setting_key_unique" UNIQUE("key")
);
--> statement-breakpoint
CREATE TABLE "hs_trial_event" (
	"id" uuid PRIMARY KEY DEFAULT gen_random_uuid() NOT NULL,
	"trial_id" uuid NOT NULL,
	"event_type" varchar(50) NOT NULL,
	"action_id" uuid,
	"timestamp" timestamp with time zone DEFAULT CURRENT_TIMESTAMP NOT NULL,
	"data" jsonb DEFAULT '{}'::jsonb,
	"created_by" uuid
);
--> statement-breakpoint
CREATE TABLE "hs_trial" (
	"id" uuid PRIMARY KEY DEFAULT gen_random_uuid() NOT NULL,
	"experiment_id" uuid NOT NULL,
	"participant_id" uuid,
	"wizard_id" uuid,
	"session_number" integer DEFAULT 1 NOT NULL,
	"status" "trial_status" DEFAULT 'scheduled' NOT NULL,
	"scheduled_at" timestamp with time zone,
	"started_at" timestamp with time zone,
	"completed_at" timestamp with time zone,
	"duration" integer,
	"notes" text,
	"parameters" jsonb DEFAULT '{}'::jsonb,
	"created_at" timestamp with time zone DEFAULT CURRENT_TIMESTAMP NOT NULL,
	"updated_at" timestamp with time zone DEFAULT CURRENT_TIMESTAMP NOT NULL,
	"metadata" jsonb DEFAULT '{}'::jsonb
);
--> statement-breakpoint
CREATE TABLE "hs_user_system_role" (
	"id" uuid PRIMARY KEY DEFAULT gen_random_uuid() NOT NULL,
	"user_id" uuid NOT NULL,
	"role" "system_role" NOT NULL,
	"granted_at" timestamp with time zone DEFAULT CURRENT_TIMESTAMP NOT NULL,
	"granted_by" uuid,
	CONSTRAINT "hs_user_system_role_user_id_role_unique" UNIQUE("user_id","role")
);
--> statement-breakpoint
CREATE TABLE "hs_user" (
	"id" uuid PRIMARY KEY DEFAULT gen_random_uuid() NOT NULL,
	"email" varchar(255) NOT NULL,
	"email_verified" timestamp with time zone,
	"name" varchar(255),
	"image" text,
	"password" varchar(255),
	"active_study_id" uuid,
	"created_at" timestamp with time zone DEFAULT CURRENT_TIMESTAMP NOT NULL,
	"updated_at" timestamp with time zone DEFAULT CURRENT_TIMESTAMP NOT NULL,
	"deleted_at" timestamp with time zone,
	CONSTRAINT "hs_user_email_unique" UNIQUE("email")
);
--> statement-breakpoint
CREATE TABLE "hs_verification_token" (
	"identifier" varchar(255) NOT NULL,
	"token" varchar(255) NOT NULL,
	"expires" timestamp with time zone NOT NULL,
	"created_at" timestamp with time zone DEFAULT CURRENT_TIMESTAMP NOT NULL,
	CONSTRAINT "hs_verification_token_identifier_token_pk" PRIMARY KEY("identifier","token"),
	CONSTRAINT "hs_verification_token_token_unique" UNIQUE("token")
);
--> statement-breakpoint
CREATE TABLE "hs_wizard_intervention" (
	"id" uuid PRIMARY KEY DEFAULT gen_random_uuid() NOT NULL,
	"trial_id" uuid NOT NULL,
	"wizard_id" uuid NOT NULL,
	"intervention_type" varchar(100) NOT NULL,
	"description" text,
	"timestamp" timestamp with time zone DEFAULT CURRENT_TIMESTAMP NOT NULL,
	"parameters" jsonb DEFAULT '{}'::jsonb,
	"reason" text
);
--> statement-breakpoint
ALTER TABLE "hs_account" ADD CONSTRAINT "hs_account_user_id_hs_user_id_fk" FOREIGN KEY ("user_id") REFERENCES "public"."hs_user"("id") ON DELETE cascade ON UPDATE no action;--> statement-breakpoint
ALTER TABLE "hs_action" ADD CONSTRAINT "hs_action_step_id_hs_step_id_fk" FOREIGN KEY ("step_id") REFERENCES "public"."hs_step"("id") ON DELETE cascade ON UPDATE no action;--> statement-breakpoint
ALTER TABLE "hs_activity_log" ADD CONSTRAINT "hs_activity_log_study_id_hs_study_id_fk" FOREIGN KEY ("study_id") REFERENCES "public"."hs_study"("id") ON DELETE cascade ON UPDATE no action;--> statement-breakpoint
ALTER TABLE "hs_activity_log" ADD CONSTRAINT "hs_activity_log_user_id_hs_user_id_fk" FOREIGN KEY ("user_id") REFERENCES "public"."hs_user"("id") ON DELETE no action ON UPDATE no action;--> statement-breakpoint
ALTER TABLE "hs_annotation" ADD CONSTRAINT "hs_annotation_trial_id_hs_trial_id_fk" FOREIGN KEY ("trial_id") REFERENCES "public"."hs_trial"("id") ON DELETE cascade ON UPDATE no action;--> statement-breakpoint
ALTER TABLE "hs_annotation" ADD CONSTRAINT "hs_annotation_annotator_id_hs_user_id_fk" FOREIGN KEY ("annotator_id") REFERENCES "public"."hs_user"("id") ON DELETE no action ON UPDATE no action;--> statement-breakpoint
ALTER TABLE "hs_attachment" ADD CONSTRAINT "hs_attachment_uploaded_by_hs_user_id_fk" FOREIGN KEY ("uploaded_by") REFERENCES "public"."hs_user"("id") ON DELETE no action ON UPDATE no action;--> statement-breakpoint
ALTER TABLE "hs_audit_log" ADD CONSTRAINT "hs_audit_log_user_id_hs_user_id_fk" FOREIGN KEY ("user_id") REFERENCES "public"."hs_user"("id") ON DELETE no action ON UPDATE no action;--> statement-breakpoint
ALTER TABLE "hs_comment" ADD CONSTRAINT "hs_comment_author_id_hs_user_id_fk" FOREIGN KEY ("author_id") REFERENCES "public"."hs_user"("id") ON DELETE no action ON UPDATE no action;--> statement-breakpoint
ALTER TABLE "hs_consent_form" ADD CONSTRAINT "hs_consent_form_study_id_hs_study_id_fk" FOREIGN KEY ("study_id") REFERENCES "public"."hs_study"("id") ON DELETE cascade ON UPDATE no action;--> statement-breakpoint
ALTER TABLE "hs_consent_form" ADD CONSTRAINT "hs_consent_form_created_by_hs_user_id_fk" FOREIGN KEY ("created_by") REFERENCES "public"."hs_user"("id") ON DELETE no action ON UPDATE no action;--> statement-breakpoint
ALTER TABLE "hs_experiment" ADD CONSTRAINT "hs_experiment_study_id_hs_study_id_fk" FOREIGN KEY ("study_id") REFERENCES "public"."hs_study"("id") ON DELETE cascade ON UPDATE no action;--> statement-breakpoint
ALTER TABLE "hs_experiment" ADD CONSTRAINT "hs_experiment_robot_id_hs_robot_id_fk" FOREIGN KEY ("robot_id") REFERENCES "public"."hs_robot"("id") ON DELETE no action ON UPDATE no action;--> statement-breakpoint
ALTER TABLE "hs_experiment" ADD CONSTRAINT "hs_experiment_created_by_hs_user_id_fk" FOREIGN KEY ("created_by") REFERENCES "public"."hs_user"("id") ON DELETE no action ON UPDATE no action;--> statement-breakpoint
ALTER TABLE "hs_export_job" ADD CONSTRAINT "hs_export_job_study_id_hs_study_id_fk" FOREIGN KEY ("study_id") REFERENCES "public"."hs_study"("id") ON DELETE cascade ON UPDATE no action;--> statement-breakpoint
ALTER TABLE "hs_export_job" ADD CONSTRAINT "hs_export_job_requested_by_hs_user_id_fk" FOREIGN KEY ("requested_by") REFERENCES "public"."hs_user"("id") ON DELETE no action ON UPDATE no action;--> statement-breakpoint
ALTER TABLE "hs_media_capture" ADD CONSTRAINT "hs_media_capture_trial_id_hs_trial_id_fk" FOREIGN KEY ("trial_id") REFERENCES "public"."hs_trial"("id") ON DELETE cascade ON UPDATE no action;--> statement-breakpoint
ALTER TABLE "hs_participant_consent" ADD CONSTRAINT "hs_participant_consent_participant_id_hs_participant_id_fk" FOREIGN KEY ("participant_id") REFERENCES "public"."hs_participant"("id") ON DELETE cascade ON UPDATE no action;--> statement-breakpoint
ALTER TABLE "hs_participant_consent" ADD CONSTRAINT "hs_participant_consent_consent_form_id_hs_consent_form_id_fk" FOREIGN KEY ("consent_form_id") REFERENCES "public"."hs_consent_form"("id") ON DELETE no action ON UPDATE no action;--> statement-breakpoint
ALTER TABLE "hs_participant" ADD CONSTRAINT "hs_participant_study_id_hs_study_id_fk" FOREIGN KEY ("study_id") REFERENCES "public"."hs_study"("id") ON DELETE cascade ON UPDATE no action;--> statement-breakpoint
ALTER TABLE "hs_plugin" ADD CONSTRAINT "hs_plugin_robot_id_hs_robot_id_fk" FOREIGN KEY ("robot_id") REFERENCES "public"."hs_robot"("id") ON DELETE cascade ON UPDATE no action;--> statement-breakpoint
ALTER TABLE "hs_role_permission" ADD CONSTRAINT "hs_role_permission_permission_id_hs_permission_id_fk" FOREIGN KEY ("permission_id") REFERENCES "public"."hs_permission"("id") ON DELETE cascade ON UPDATE no action;--> statement-breakpoint
ALTER TABLE "hs_sensor_data" ADD CONSTRAINT "hs_sensor_data_trial_id_hs_trial_id_fk" FOREIGN KEY ("trial_id") REFERENCES "public"."hs_trial"("id") ON DELETE cascade ON UPDATE no action;--> statement-breakpoint
ALTER TABLE "hs_session" ADD CONSTRAINT "hs_session_user_id_hs_user_id_fk" FOREIGN KEY ("user_id") REFERENCES "public"."hs_user"("id") ON DELETE cascade ON UPDATE no action;--> statement-breakpoint
ALTER TABLE "hs_shared_resource" ADD CONSTRAINT "hs_shared_resource_study_id_hs_study_id_fk" FOREIGN KEY ("study_id") REFERENCES "public"."hs_study"("id") ON DELETE cascade ON UPDATE no action;--> statement-breakpoint
ALTER TABLE "hs_shared_resource" ADD CONSTRAINT "hs_shared_resource_shared_by_hs_user_id_fk" FOREIGN KEY ("shared_by") REFERENCES "public"."hs_user"("id") ON DELETE no action ON UPDATE no action;--> statement-breakpoint
ALTER TABLE "hs_step" ADD CONSTRAINT "hs_step_experiment_id_hs_experiment_id_fk" FOREIGN KEY ("experiment_id") REFERENCES "public"."hs_experiment"("id") ON DELETE cascade ON UPDATE no action;--> statement-breakpoint
ALTER TABLE "hs_study" ADD CONSTRAINT "hs_study_created_by_hs_user_id_fk" FOREIGN KEY ("created_by") REFERENCES "public"."hs_user"("id") ON DELETE no action ON UPDATE no action;--> statement-breakpoint
ALTER TABLE "hs_study_member" ADD CONSTRAINT "hs_study_member_study_id_hs_study_id_fk" FOREIGN KEY ("study_id") REFERENCES "public"."hs_study"("id") ON DELETE cascade ON UPDATE no action;--> statement-breakpoint
ALTER TABLE "hs_study_member" ADD CONSTRAINT "hs_study_member_user_id_hs_user_id_fk" FOREIGN KEY ("user_id") REFERENCES "public"."hs_user"("id") ON DELETE cascade ON UPDATE no action;--> statement-breakpoint
ALTER TABLE "hs_study_member" ADD CONSTRAINT "hs_study_member_invited_by_hs_user_id_fk" FOREIGN KEY ("invited_by") REFERENCES "public"."hs_user"("id") ON DELETE no action ON UPDATE no action;--> statement-breakpoint
ALTER TABLE "hs_study_plugin" ADD CONSTRAINT "hs_study_plugin_study_id_hs_study_id_fk" FOREIGN KEY ("study_id") REFERENCES "public"."hs_study"("id") ON DELETE cascade ON UPDATE no action;--> statement-breakpoint
ALTER TABLE "hs_study_plugin" ADD CONSTRAINT "hs_study_plugin_plugin_id_hs_plugin_id_fk" FOREIGN KEY ("plugin_id") REFERENCES "public"."hs_plugin"("id") ON DELETE no action ON UPDATE no action;--> statement-breakpoint
ALTER TABLE "hs_study_plugin" ADD CONSTRAINT "hs_study_plugin_installed_by_hs_user_id_fk" FOREIGN KEY ("installed_by") REFERENCES "public"."hs_user"("id") ON DELETE no action ON UPDATE no action;--> statement-breakpoint
ALTER TABLE "hs_system_setting" ADD CONSTRAINT "hs_system_setting_updated_by_hs_user_id_fk" FOREIGN KEY ("updated_by") REFERENCES "public"."hs_user"("id") ON DELETE no action ON UPDATE no action;--> statement-breakpoint
ALTER TABLE "hs_trial_event" ADD CONSTRAINT "hs_trial_event_trial_id_hs_trial_id_fk" FOREIGN KEY ("trial_id") REFERENCES "public"."hs_trial"("id") ON DELETE cascade ON UPDATE no action;--> statement-breakpoint
ALTER TABLE "hs_trial_event" ADD CONSTRAINT "hs_trial_event_action_id_hs_action_id_fk" FOREIGN KEY ("action_id") REFERENCES "public"."hs_action"("id") ON DELETE no action ON UPDATE no action;--> statement-breakpoint
ALTER TABLE "hs_trial_event" ADD CONSTRAINT "hs_trial_event_created_by_hs_user_id_fk" FOREIGN KEY ("created_by") REFERENCES "public"."hs_user"("id") ON DELETE no action ON UPDATE no action;--> statement-breakpoint
ALTER TABLE "hs_trial" ADD CONSTRAINT "hs_trial_experiment_id_hs_experiment_id_fk" FOREIGN KEY ("experiment_id") REFERENCES "public"."hs_experiment"("id") ON DELETE no action ON UPDATE no action;--> statement-breakpoint
ALTER TABLE "hs_trial" ADD CONSTRAINT "hs_trial_participant_id_hs_participant_id_fk" FOREIGN KEY ("participant_id") REFERENCES "public"."hs_participant"("id") ON DELETE no action ON UPDATE no action;--> statement-breakpoint
ALTER TABLE "hs_trial" ADD CONSTRAINT "hs_trial_wizard_id_hs_user_id_fk" FOREIGN KEY ("wizard_id") REFERENCES "public"."hs_user"("id") ON DELETE no action ON UPDATE no action;--> statement-breakpoint
ALTER TABLE "hs_user_system_role" ADD CONSTRAINT "hs_user_system_role_user_id_hs_user_id_fk" FOREIGN KEY ("user_id") REFERENCES "public"."hs_user"("id") ON DELETE cascade ON UPDATE no action;--> statement-breakpoint
ALTER TABLE "hs_user_system_role" ADD CONSTRAINT "hs_user_system_role_granted_by_hs_user_id_fk" FOREIGN KEY ("granted_by") REFERENCES "public"."hs_user"("id") ON DELETE no action ON UPDATE no action;--> statement-breakpoint
ALTER TABLE "hs_user" ADD CONSTRAINT "hs_user_active_study_id_hs_study_id_fk" FOREIGN KEY ("active_study_id") REFERENCES "public"."hs_study"("id") ON DELETE no action ON UPDATE no action;--> statement-breakpoint
ALTER TABLE "hs_wizard_intervention" ADD CONSTRAINT "hs_wizard_intervention_trial_id_hs_trial_id_fk" FOREIGN KEY ("trial_id") REFERENCES "public"."hs_trial"("id") ON DELETE cascade ON UPDATE no action;--> statement-breakpoint
ALTER TABLE "hs_wizard_intervention" ADD CONSTRAINT "hs_wizard_intervention_wizard_id_hs_user_id_fk" FOREIGN KEY ("wizard_id") REFERENCES "public"."hs_user"("id") ON DELETE no action ON UPDATE no action;--> statement-breakpoint
CREATE INDEX "account_user_id_idx" ON "hs_account" USING btree ("user_id");--> statement-breakpoint
CREATE INDEX "activity_logs_study_created_idx" ON "hs_activity_log" USING btree ("study_id","created_at");--> statement-breakpoint
CREATE INDEX "audit_logs_created_idx" ON "hs_audit_log" USING btree ("created_at");--> statement-breakpoint
CREATE INDEX "sensor_data_trial_timestamp_idx" ON "hs_sensor_data" USING btree ("trial_id","timestamp");--> statement-breakpoint
CREATE INDEX "session_user_id_idx" ON "hs_session" USING btree ("user_id");--> statement-breakpoint
CREATE INDEX "trial_events_trial_timestamp_idx" ON "hs_trial_event" USING btree ("trial_id","timestamp");