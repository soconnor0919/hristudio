import { NextResponse } from "next/server";
import { db } from "~/server/db";
import { contents, informedConsentForms, contentTypes } from "~/server/db/schema";
import { auth } from "@clerk/nextjs/server";
import { eq } from "drizzle-orm";
import { saveFile } from "~/lib/fileStorage";
import fs from 'fs/promises';
import { studies, participants } from "~/server/db/schema";
import { anonymizeParticipants } from "~/lib/permissions"; // Import the anonymize function

// Function to generate a random string
const generateRandomString = (length: number) => {
  const characters = 'abcdefghijklmnopqrstuvwxyz0123456789';
  let result = '';
  for (let i = 0; i < length; i++) {
    result += characters.charAt(Math.floor(Math.random() * characters.length));
  }
  return result;
};

export async function GET(request: Request) {
  const { userId } = auth();
  if (!userId) {
    return NextResponse.json({ error: 'Unauthorized' }, { status: 401 });
  }

  const forms = await db.select({
    id: informedConsentForms.id,
    title: contents.title,
    location: contents.location,
    previewLocation: contents.previewLocation,
    studyId: informedConsentForms.studyId,
    studyTitle: studies.title,
    participantId: informedConsentForms.participantId,
    participantName: participants.name,
    contentId: informedConsentForms.contentId,
  }).from(informedConsentForms)
    .innerJoin(contents, eq(informedConsentForms.contentId, contents.id))
    .innerJoin(studies, eq(informedConsentForms.studyId, studies.id))
    .innerJoin(participants, eq(informedConsentForms.participantId, participants.id));

  // Anonymize participant names
  const anonymizedForms = forms.map(form => ({
    ...form,
    participantName: `Participant ${form.participantId}` // Anonymizing logic
  }));

  return NextResponse.json(anonymizedForms);
}

export async function POST(request: Request) {
  const { userId } = auth();
  if (!userId) {
    return NextResponse.json({ error: 'Unauthorized' }, { status: 401 });
  }

  const formData = await request.formData();
  const file = formData.get('file') as File;
  const title = formData.get('title') as string;
  const studyId = formData.get('studyId') as string;
  const participantId = formData.get('participantId') as string;

  if (!file || !title || !studyId || !participantId) {
    return NextResponse.json({ error: 'Missing required fields' }, { status: 400 });
  }

  try {
    const [formContentType] = await db
      .select()
      .from(contentTypes)
      .where(eq(contentTypes.name, "Informed Consent Form"));

    const [previewContentType] = await db
      .select()
      .from(contentTypes)
      .where(eq(contentTypes.name, "Preview Image"));

    if (!formContentType || !previewContentType) {
      return NextResponse.json({ error: 'Content type not found' }, { status: 500 });
    }

    // Generate a random filename with the same extension
    const fileExtension = file.name.split('.').pop(); // Get the file extension
    const randomFileName = `${generateRandomString(12)}.${fileExtension}`; // Generate random filename with 12 characters
    const { pdfPath, previewPath } = await saveFile(file, `${formContentType.id}/${randomFileName}`, previewContentType.id);

    const [content] = await db
      .insert(contents)
      .values({
        contentTypeId: formContentType.id,
        uploader: userId,
        location: pdfPath,
        previewLocation: previewPath,
        title: title,
      })
      .returning();

    if (!content) {
      throw new Error("Content not found");
    }

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
    console.error('Error uploading form:', error);
    return NextResponse.json({ error: 'Failed to upload form' }, { status: 500 });
  }
}