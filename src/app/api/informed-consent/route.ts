import { NextResponse } from "next/server";
import { db } from "~/server/db";
import { contents, informedConsentForms, contentTypes } from "~/server/db/schema";
import { auth } from "@clerk/nextjs/server";
import { eq } from "drizzle-orm";
import { saveFile } from '~/lib/fileStorage';

export async function POST(request: Request) {
  const { userId } = auth();
  if (!userId) {
    return NextResponse.json({ error: 'Unauthorized' }, { status: 401 });
  }

  const formData = await request.formData();
  const file = formData.get('file') as File;
  const studyId = formData.get('studyId') as string;
  const participantId = formData.get('participantId') as string;

  if (!file || !studyId || !participantId) {
    return NextResponse.json({ error: 'Missing required fields' }, { status: 400 });
  }

  try {
    const [contentType] = await db
      .select()
      .from(contentTypes)
      .where(eq(contentTypes.name, "Informed Consent Form"));

    if (!contentType) {
      return NextResponse.json({ error: 'Content type not found' }, { status: 500 });
    }

    const [content] = await db
      .insert(contents)
      .values({
        contentTypeId: contentType.id,
        uploader: userId,
        location: '', // We'll update this after saving the file
      })
      .returning();

    if (!content) {
      throw new Error("Content not found");
    }

    const fileLocation = await saveFile(file, content.id);

    await db
      .update(contents)
      .set({ location: fileLocation })
      .where(eq(contents.id, content.id));

    const [form] = await db
      .insert(informedConsentForms)
      .values({
        studyId: parseInt(studyId),
        participantId: parseInt(participantId),
        contentId: content.id,
      })
      .returning();

    return NextResponse.json(form);
  } catch (error) {
    console.error('Error uploading informed consent form:', error);
    return NextResponse.json({ error: 'Failed to upload form' }, { status: 500 });
  }
}