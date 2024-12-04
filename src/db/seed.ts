import { config } from "dotenv";
import { eq } from "drizzle-orm";
import { db } from "./index";
import { PERMISSIONS } from "~/lib/permissions";
import { ROLES, ROLE_PERMISSIONS } from "~/lib/roles";
import { permissionsTable, rolesTable, rolePermissionsTable, userRolesTable, usersTable, studyTable } from "./schema";

// Load environment variables from .env.local
config({ path: ".env.local" });

async function seed() {
  console.log("🌱 Seeding database...");

  // Insert roles
  console.log("Inserting roles...");
  for (const [roleKey, roleName] of Object.entries(ROLES)) {
    await db.insert(rolesTable)
      .values({
        name: roleName,
        description: getRoleDescription(roleKey),
      })
      .onConflictDoNothing();
  }

  // Insert permissions
  console.log("Inserting permissions...");
  for (const [permKey, permCode] of Object.entries(PERMISSIONS)) {
    await db.insert(permissionsTable)
      .values({
        name: formatPermissionName(permKey),
        code: permCode,
        description: getPermissionDescription(permKey),
      })
      .onConflictDoNothing();
  }

  // Get role and permission IDs
  const roles = await db.select().from(rolesTable);
  const permissions = await db.select().from(permissionsTable);

  // Insert role permissions
  console.log("Inserting role permissions...");
  for (const [roleKey, permissionCodes] of Object.entries(ROLE_PERMISSIONS)) {
    const role = roles.find(r => r.name === ROLES[roleKey as keyof typeof ROLES]);
    if (!role) continue;

    for (const permissionCode of permissionCodes) {
      const permission = permissions.find(p => p.code === PERMISSIONS[permissionCode]);
      if (!permission) continue;

      await db.insert(rolePermissionsTable)
        .values({
          roleId: role.id,
          permissionId: permission.id,
        })
        .onConflictDoNothing();
    }
  }

  // Get the first user and assign them as a Principal Investigator for their studies
  console.log("Setting up initial user roles...");
  const users = await db.select().from(usersTable);
  if (users.length > 0) {
    const piRole = roles.find(r => r.name === ROLES.PRINCIPAL_INVESTIGATOR);
    if (piRole) {
      // Get all studies owned by the first user
      const userStudies = await db
        .select()
        .from(studyTable)
        .where(eq(studyTable.userId, users[0].id));

      // Assign PI role for each study
      for (const study of userStudies) {
        await db.insert(userRolesTable)
          .values({
            userId: users[0].id,
            roleId: piRole.id,
            studyId: study.id,
          })
          .onConflictDoNothing();
      }
    }
  }

  console.log("✅ Seeding complete!");
}

function getRoleDescription(roleKey: string): string {
  const descriptions: Record<string, string> = {
    ADMIN: "Full system administrator with all permissions",
    PRINCIPAL_INVESTIGATOR: "Lead researcher responsible for study design and oversight",
    RESEARCHER: "Study team member with data collection and analysis capabilities",
    WIZARD: "Operator controlling robot behavior during experiments",
    OBSERVER: "Team member observing and annotating experiments",
    ASSISTANT: "Support staff with limited view access",
  };
  return descriptions[roleKey] || "";
}

function getPermissionDescription(permKey: string): string {
  const descriptions: Record<string, string> = {
    CREATE_STUDY: "Create new research studies",
    EDIT_STUDY: "Modify existing study parameters",
    DELETE_STUDY: "Remove studies from the system",
    VIEW_STUDY: "View study details and progress",
    VIEW_PARTICIPANT_NAMES: "Access participant identifying information",
    CREATE_PARTICIPANT: "Add new participants to studies",
    EDIT_PARTICIPANT: "Update participant information",
    DELETE_PARTICIPANT: "Remove participants from studies",
    CONTROL_ROBOT: "Operate robot during experiments",
    VIEW_ROBOT_STATUS: "Monitor robot state and sensors",
    RECORD_EXPERIMENT: "Start/stop experiment recording",
    VIEW_EXPERIMENT: "View experiment progress and details",
    VIEW_EXPERIMENT_DATA: "Access collected experiment data",
    EXPORT_EXPERIMENT_DATA: "Download experiment data",
    ANNOTATE_EXPERIMENT: "Add notes and annotations to experiments",
    MANAGE_ROLES: "Assign and modify user roles",
    MANAGE_USERS: "Add and remove system users",
    MANAGE_SYSTEM_SETTINGS: "Configure system-wide settings",
  };
  return descriptions[permKey] || "";
}

function formatPermissionName(permKey: string): string {
  return permKey.toLowerCase()
    .split('_')
    .map(word => word.charAt(0).toUpperCase() + word.slice(1))
    .join(' ');
}

seed().catch(console.error);
