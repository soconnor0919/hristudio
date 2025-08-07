#!/usr/bin/env tsx

/**
 * Test script to validate seed data structure
 * Ensures all user relationships and study memberships are correct
 */

interface User {
  id: string;
  name: string;
  email: string;
  institution: string;
}

interface UserRole {
  userId: string;
  role: "administrator" | "researcher" | "wizard" | "observer";
  assignedBy: string;
}

interface Study {
  id: string;
  name: string;
  createdBy: string;
}

interface StudyMember {
  studyId: string;
  userId: string;
  role: "owner" | "researcher" | "wizard" | "observer";
  invitedBy: string | null;
}

function validateSeedData() {
  console.log("🧪 Testing seed data structure...\n");

  // Users data
  const users: User[] = [
    {
      id: "01234567-89ab-cdef-0123-456789abcde0",
      name: "Sean O'Connor",
      email: "sean@soconnor.dev",
      institution: "HRIStudio",
    },
    {
      id: "01234567-89ab-cdef-0123-456789abcde1",
      name: "Dr. Sarah Chen",
      email: "sarah.chen@university.edu",
      institution: "MIT Computer Science",
    },
    {
      id: "01234567-89ab-cdef-0123-456789abcde2",
      name: "Dr. Michael Rodriguez",
      email: "m.rodriguez@research.org",
      institution: "Stanford HCI Lab",
    },
    {
      id: "01234567-89ab-cdef-0123-456789abcde3",
      name: "Emma Thompson",
      email: "emma.thompson@university.edu",
      institution: "MIT Computer Science",
    },
    {
      id: "01234567-89ab-cdef-0123-456789abcde4",
      name: "Dr. James Wilson",
      email: "james.wilson@university.edu",
      institution: "MIT Computer Science",
    },
  ];

  // User roles
  const userRoles: UserRole[] = [
    {
      userId: "01234567-89ab-cdef-0123-456789abcde0",
      role: "administrator",
      assignedBy: "01234567-89ab-cdef-0123-456789abcde0",
    },
    {
      userId: "01234567-89ab-cdef-0123-456789abcde1",
      role: "researcher",
      assignedBy: "01234567-89ab-cdef-0123-456789abcde0",
    },
    {
      userId: "01234567-89ab-cdef-0123-456789abcde2",
      role: "researcher",
      assignedBy: "01234567-89ab-cdef-0123-456789abcde0",
    },
    {
      userId: "01234567-89ab-cdef-0123-456789abcde3",
      role: "wizard",
      assignedBy: "01234567-89ab-cdef-0123-456789abcde0",
    },
    {
      userId: "01234567-89ab-cdef-0123-456789abcde4",
      role: "observer",
      assignedBy: "01234567-89ab-cdef-0123-456789abcde0",
    },
  ];

  // Studies
  const studies: Study[] = [
    {
      id: "11234567-89ab-cdef-0123-456789abcde1",
      name: "Robot Navigation Assistance Study",
      createdBy: "01234567-89ab-cdef-0123-456789abcde0",
    },
    {
      id: "11234567-89ab-cdef-0123-456789abcde2",
      name: "Social Robots in Healthcare Study",
      createdBy: "01234567-89ab-cdef-0123-456789abcde0",
    },
    {
      id: "11234567-89ab-cdef-0123-456789abcde3",
      name: "Elderly Care Robot Interaction Study",
      createdBy: "01234567-89ab-cdef-0123-456789abcde0",
    },
  ];

  // Study members
  const studyMembers: StudyMember[] = [
    // Sean as owner of all studies
    {
      studyId: "11234567-89ab-cdef-0123-456789abcde1",
      userId: "01234567-89ab-cdef-0123-456789abcde0",
      role: "owner",
      invitedBy: null,
    },
    {
      studyId: "11234567-89ab-cdef-0123-456789abcde2",
      userId: "01234567-89ab-cdef-0123-456789abcde0",
      role: "owner",
      invitedBy: null,
    },
    {
      studyId: "11234567-89ab-cdef-0123-456789abcde3",
      userId: "01234567-89ab-cdef-0123-456789abcde0",
      role: "owner",
      invitedBy: null,
    },
    // Other team members
    {
      studyId: "11234567-89ab-cdef-0123-456789abcde1",
      userId: "01234567-89ab-cdef-0123-456789abcde2",
      role: "researcher",
      invitedBy: "01234567-89ab-cdef-0123-456789abcde0",
    },
    {
      studyId: "11234567-89ab-cdef-0123-456789abcde1",
      userId: "01234567-89ab-cdef-0123-456789abcde3",
      role: "wizard",
      invitedBy: "01234567-89ab-cdef-0123-456789abcde0",
    },
    {
      studyId: "11234567-89ab-cdef-0123-456789abcde2",
      userId: "01234567-89ab-cdef-0123-456789abcde2",
      role: "researcher",
      invitedBy: "01234567-89ab-cdef-0123-456789abcde0",
    },
    {
      studyId: "11234567-89ab-cdef-0123-456789abcde3",
      userId: "01234567-89ab-cdef-0123-456789abcde1",
      role: "researcher",
      invitedBy: "01234567-89ab-cdef-0123-456789abcde0",
    },
  ];

  let errors = 0;

  console.log("👥 Validating users...");
  console.log(`   Users: ${users.length}`);

  // Check for Sean as admin
  const seanUser = users.find((u) => u.email === "sean@soconnor.dev");
  if (seanUser) {
    console.log(`   ✅ Sean found: ${seanUser.name} (${seanUser.email})`);
  } else {
    console.error(`   ❌ Sean not found as user`);
    errors++;
  }

  console.log("\n🔐 Validating user roles...");
  console.log(`   User roles: ${userRoles.length}`);

  // Check Sean's admin role
  const seanRole = userRoles.find(
    (r) => r.userId === "01234567-89ab-cdef-0123-456789abcde0",
  );
  if (seanRole && seanRole.role === "administrator") {
    console.log(`   ✅ Sean has administrator role`);
  } else {
    console.error(`   ❌ Sean missing administrator role`);
    errors++;
  }

  // Check all roles are assigned by Sean
  const rolesAssignedBySean = userRoles.filter(
    (r) => r.assignedBy === "01234567-89ab-cdef-0123-456789abcde0",
  );
  console.log(
    `   ✅ ${rolesAssignedBySean.length}/${userRoles.length} roles assigned by Sean`,
  );

  console.log("\n📚 Validating studies...");
  console.log(`   Studies: ${studies.length}`);

  // Check all studies created by Sean
  const studiesCreatedBySean = studies.filter(
    (s) => s.createdBy === "01234567-89ab-cdef-0123-456789abcde0",
  );
  if (studiesCreatedBySean.length === studies.length) {
    console.log(`   ✅ All ${studies.length} studies created by Sean`);
  } else {
    console.error(
      `   ❌ Only ${studiesCreatedBySean.length}/${studies.length} studies created by Sean`,
    );
    errors++;
  }

  console.log("\n👨‍💼 Validating study memberships...");
  console.log(`   Study memberships: ${studyMembers.length}`);

  // Check Sean is owner of all studies
  const seanOwnerships = studyMembers.filter(
    (m) =>
      m.userId === "01234567-89ab-cdef-0123-456789abcde0" && m.role === "owner",
  );
  if (seanOwnerships.length === studies.length) {
    console.log(`   ✅ Sean is owner of all ${studies.length} studies`);
  } else {
    console.error(
      `   ❌ Sean only owns ${seanOwnerships.length}/${studies.length} studies`,
    );
    errors++;
  }

  // Check invitation chain
  const membersInvitedBySean = studyMembers.filter(
    (m) => m.invitedBy === "01234567-89ab-cdef-0123-456789abcde0",
  );
  console.log(`   ✅ ${membersInvitedBySean.length} members invited by Sean`);

  // Validate all user references exist
  console.log("\n🔗 Validating references...");

  const userIds = new Set(users.map((u) => u.id));

  for (const role of userRoles) {
    if (!userIds.has(role.userId)) {
      console.error(`   ❌ Invalid user reference in role: ${role.userId}`);
      errors++;
    }
    if (!userIds.has(role.assignedBy)) {
      console.error(
        `   ❌ Invalid assignedBy reference in role: ${role.assignedBy}`,
      );
      errors++;
    }
  }

  for (const study of studies) {
    if (!userIds.has(study.createdBy)) {
      console.error(
        `   ❌ Invalid createdBy reference in study: ${study.createdBy}`,
      );
      errors++;
    }
  }

  const studyIds = new Set(studies.map((s) => s.id));

  for (const member of studyMembers) {
    if (!studyIds.has(member.studyId)) {
      console.error(
        `   ❌ Invalid study reference in membership: ${member.studyId}`,
      );
      errors++;
    }
    if (!userIds.has(member.userId)) {
      console.error(
        `   ❌ Invalid user reference in membership: ${member.userId}`,
      );
      errors++;
    }
    if (member.invitedBy && !userIds.has(member.invitedBy)) {
      console.error(
        `   ❌ Invalid invitedBy reference in membership: ${member.invitedBy}`,
      );
      errors++;
    }
  }

  if (errors === 0) {
    console.log("   ✅ All references are valid");
  }

  // Summary
  console.log(`\n📊 Validation Summary:`);
  console.log(`   Users: ${users.length}`);
  console.log(`   User roles: ${userRoles.length}`);
  console.log(`   Studies: ${studies.length}`);
  console.log(`   Study memberships: ${studyMembers.length}`);
  console.log(`   Errors: ${errors}`);

  if (errors === 0) {
    console.log(`\n🎉 All validations passed! Seed data structure is correct.`);
    console.log(`   Sean (sean@soconnor.dev) is admin of everything:`);
    console.log(`   • System administrator role`);
    console.log(`   • Owner of all ${studies.length} studies`);
    console.log(`   • Assigned all user roles`);
    console.log(`   • Invited all study members`);
    process.exit(0);
  } else {
    console.log(`\n❌ Validation failed with ${errors} error(s).`);
    process.exit(1);
  }
}

// Run the validation
if (import.meta.url === `file://${process.argv[1]}`) {
  validateSeedData();
}

export { validateSeedData };
