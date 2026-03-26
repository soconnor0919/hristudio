-- Migration 0001: Minimal Seed Data
-- HRIStudio - Only essential data needed for auth

-- ======================
-- USERS & AUTH
-- ======================

-- Users (using valid UUID v4 format)
INSERT INTO "hs_user" ("id", "name", "email", "email_verified", "image", "created_at", "updated_at")
VALUES 
  ('11111111-1111-4111-8111-111111111111', 'Sean O''Connor', 'sean@soconnor.dev', true, 'https://www.gravatar.com/avatar/4b20f4a15f9a0e0f5e5e5a0f5e5e5a0f?d=identicon', NOW(), NOW()),
  ('22222222-2222-4222-8222-222222222222', 'Dr. Felipe Perrone', 'felipe.perrone@bucknell.edu', true, 'https://api.dicebear.com/7.x/avataaars/svg?seed=Felipe', NOW(), NOW())
ON CONFLICT DO NOTHING;

-- Accounts
INSERT INTO "hs_account" ("id", "user_id", "provider_id", "account_id", "password", "created_at", "updated_at")
VALUES 
  ('aaaaaaaa-aaaa-4aaa-aaaa-aaaaaaaaaaaa', '11111111-1111-4111-8111-111111111111', 'credential', '11111111-1111-4111-8111-111111111111', '$2b$12$50kgpkp.qZrZXCWjHuVSHOZBjAQUrX50VdtWc6WBj27HnzUYFwwPm', NOW(), NOW()),
  ('bbbbbbbb-bbbb-4bbb-bbbb-bbbbbbbbbbbb', '22222222-2222-4222-8222-222222222222', 'credential', '22222222-2222-4222-8222-222222222222', '$2b$12$50kgpkp.qZrZXCWjHuVSHOZBjAQUrX50VdtWc6WBj27HnzUYFwwPm', NOW(), NOW())
ON CONFLICT DO NOTHING;

-- System Roles
INSERT INTO "hs_user_system_role" ("id", "user_id", "role", "granted_at", "granted_by")
VALUES 
  (gen_random_uuid(), '11111111-1111-4111-8111-111111111111', 'administrator', NOW(), '11111111-1111-4111-8111-111111111111'),
  (gen_random_uuid(), '22222222-2222-4222-8222-222222222222', 'researcher', NOW(), '11111111-1111-4111-8111-111111111111')
ON CONFLICT DO NOTHING;

DO $$
BEGIN
  RAISE NOTICE 'Minimal seed migration complete';
  RAISE NOTICE 'Admin: sean@soconnor.dev / password123';
  RAISE NOTICE 'Use bun db:seed for full demo data';
END $$;