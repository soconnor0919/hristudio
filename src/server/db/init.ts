import { db } from "./index";
import { contentTypes } from "./schema";
import { roles } from "./schema";

export async function initializeContentTypes() {
  const existingTypes = await db.select().from(contentTypes);
  
  if (existingTypes.length === 0) {
    await db.insert(contentTypes).values([
      { name: "Informed Consent Form" },
      { name: "Preview Image" }, // New content type
      // Add other content types as needed
    ]);
    console.log("Content types initialized");
  } else {
    console.log("Content types already initialized");
  }
}

export async function initializeRoles() {
  const existingRoles = await db.select().from(roles);

  if (existingRoles.length === 0) {
    await db.insert(roles).values([
      { name: "Basic User" }, // Role ID 0
      { name: "Admin" },      // Role ID 1
    ]);
    console.log("Roles initialized");
  } else {
    console.log("Roles already initialized");
  }
}
