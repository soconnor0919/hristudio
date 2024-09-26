import { NextResponse } from "next/server";
import { db } from "~/server/db";
import { informedConsentForms, contents } from "~/server/db/schema";
import { auth } from "@clerk/nextjs/server";
import { eq } from "drizzle-orm";
import fs from 'fs/promises';
import path from 'path';

export async function DELETE(
  request: Request,
  { params }: { params: { id: string } }
) {
  const { userId } = auth();
  if (!userId) {
    return NextResponse.json({ error: 'Unauthorized' }, { status: 401 });
  }

  const id = parseInt(params.id);
  if (isNaN(id)) {
    return NextResponse.json({ error: 'Invalid ID' }, { status: 400 });
  }

  try {
    // First, get the content associated with this form
    const [form] = await db
      .select({
        contentId: informedConsentForms.contentId,
        location: contents.location,
      })
      .from(informedConsentForms)
      .innerJoin(contents, eq(informedConsentForms.contentId, contents.id))
      .where(eq(informedConsentForms.id, id));

    if (!form) {
      return NextResponse.json({ error: 'Form not found' }, { status: 404 });
    }

    // Delete the file from the file system
    const fullPath = path.join(process.cwd(), form.location);
    try {
      await fs.access(fullPath);
      await fs.unlink(fullPath);
    } catch (error) {
      console.warn(`File not found or couldn't be deleted: ${fullPath}`);
    }

    // Delete the form and content from the database
    await db.transaction(async (tx) => {
      await tx.delete(informedConsentForms).where(eq(informedConsentForms.id, id));
      await tx.delete(contents).where(eq(contents.id, form.contentId));
    });

    return NextResponse.json({ message: "Form deleted successfully" });
  } catch (error) {
    console.error('Error deleting form:', error);
    return NextResponse.json({ error: 'Failed to delete form' }, { status: 500 });
  }
}