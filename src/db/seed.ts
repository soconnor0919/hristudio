import { db } from "~/db";
import {
  permissionsTable,
  rolesTable,
  rolePermissionsTable,
} from "~/db/schema";
import { config } from "dotenv";

config({ path: ".env.local" });

async function seed() {
  try {
    console.log("Starting seed...");

    // Create permissions
    const createdPermissions = await db
      .insert(permissionsTable)
      .values([
        {
          name: "View Participant Names",
          code: "view_participant_names",
          description: "Can view participant names",
        },
        {
          name: "Create Participant",
          code: "create_participant",
          description: "Can create new participants",
        },
        {
          name: "Delete Participant",
          code: "delete_participant",
          description: "Can delete participants",
        },
        {
          name: "Create Study",
          code: "create_study",
          description: "Can create new studies",
        },
        {
          name: "Delete Study",
          code: "delete_study",
          description: "Can delete studies",
        },
        {
          name: "Manage Roles",
          code: "manage_roles",
          description: "Can manage user roles",
        },
      ])
      .returning();

    console.log("Created permissions:", createdPermissions);

    // Create roles
    const createdRoles = await db
      .insert(rolesTable)
      .values([
        {
          name: "Admin",
          description: "Full system access",
        },
        {
          name: "Researcher",
          description: "Can manage studies and participants",
        },
        {
          name: "Observer",
          description: "Can view participant names only",
        },
      ])
      .returning();

    console.log("Created roles:", createdRoles);

    // Find roles by name
    const adminRole = createdRoles.find((r) => r.name === "Admin");
    const researcherRole = createdRoles.find((r) => r.name === "Researcher");
    const observerRole = createdRoles.find((r) => r.name === "Observer");

    // Assign permissions to roles
    if (adminRole) {
      // Admin gets all permissions
      await db.insert(rolePermissionsTable).values(
        createdPermissions.map((p) => ({
          roleId: adminRole.id,
          permissionId: p.id,
        }))
      );
      console.log("Assigned all permissions to Admin role");
    }

    if (researcherRole) {
      // Researcher gets specific permissions
      const researcherPermissions = createdPermissions.filter((p) =>
        [
          "view_participant_names",
          "create_participant",
          "create_study",
        ].includes(p.code)
      );

      await db.insert(rolePermissionsTable).values(
        researcherPermissions.map((p) => ({
          roleId: researcherRole.id,
          permissionId: p.id,
        }))
      );
      console.log("Assigned permissions to Researcher role");
    }

    if (observerRole) {
      // Observer gets view-only permissions
      const observerPermissions = createdPermissions.filter((p) =>
        ["view_participant_names"].includes(p.code)
      );

      await db.insert(rolePermissionsTable).values(
        observerPermissions.map((p) => ({
          roleId: observerRole.id,
          permissionId: p.id,
        }))
      );
      console.log("Assigned permissions to Observer role");
    }

    console.log("Seeding completed successfully");
  } catch (error) {
    console.error("Error seeding database:", error);
    throw error;
  }
}

seed()
  .catch(console.error)
  .finally(() => process.exit());
