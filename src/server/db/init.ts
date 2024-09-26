import { db } from "./index";
import { contentTypes } from "./schema";

export async function initializeContentTypes() {
  const existingTypes = await db.select().from(contentTypes);
  
  if (existingTypes.length === 0) {
    await db.insert(contentTypes).values([
      { name: "Informed Consent Form" },
      // Add other content types as needed
    ]);
    console.log("Content types initialized");
  } else {
    console.log("Content types already exist");
  }
}
