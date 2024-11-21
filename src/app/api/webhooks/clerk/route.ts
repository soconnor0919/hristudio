import { Webhook } from 'svix';
import { headers } from 'next/headers';
import { WebhookEvent } from '@clerk/nextjs/server';
import { db } from '~/db';
import { usersTable, rolesTable, userRolesTable } from '~/db/schema';
import { eq } from 'drizzle-orm';

export async function POST(req: Request) {
  const WEBHOOK_SECRET = process.env.CLERK_WEBHOOK_SECRET;
  
  if (!WEBHOOK_SECRET) {
    throw new Error('Please add CLERK_WEBHOOK_SECRET from Clerk Dashboard to .env');
  }

  // Get the headers
  const headersList = await headers();
  const svix_id = headersList.get("svix-id");
  const svix_timestamp = headersList.get("svix-timestamp");
  const svix_signature = headersList.get("svix-signature");

  if (!svix_id || !svix_timestamp || !svix_signature) {
    return new Response('Error occurred -- no svix headers', {
      status: 400
    });
  }

  // Get the body
  const payload = await req.json();
  const body = JSON.stringify(payload);

  // Verify the webhook
  const webhook = new Webhook(WEBHOOK_SECRET);
  let event: WebhookEvent;

  try {
    event = webhook.verify(body, {
      "svix-id": svix_id,
      "svix-timestamp": svix_timestamp,
      "svix-signature": svix_signature,
    }) as WebhookEvent;
  } catch (err) {
    console.error('Error verifying webhook:', err);
    return new Response('Error occurred', {
      status: 400
    });
  }

  const eventType = event.type;

  if (eventType === 'user.created' || eventType === 'user.updated') {
    const { id, first_name, last_name, email_addresses, image_url } = event.data;
    const primaryEmail = email_addresses?.[0]?.email_address;

    if (!primaryEmail) {
      return new Response('No email found', { status: 400 });
    }

    try {
      // Combine first and last name
      const fullName = [first_name, last_name].filter(Boolean).join(' ');

      // Create/update user
      await db
        .insert(usersTable)
        .values({
          id,
          name: fullName,
          email: primaryEmail,
          imageUrl: image_url,
        })
        .onConflictDoUpdate({
          target: usersTable.id,
          set: {
            name: fullName,
            email: primaryEmail,
            imageUrl: image_url,
            updatedAt: new Date(),
          },
        });

      // Assign default role (Observer)
      const observerRole = await db
        .select()
        .from(rolesTable)
        .where(eq(rolesTable.name, 'Observer'))
        .limit(1);

      if (observerRole[0]) {
        await db
          .insert(userRolesTable)
          .values({
            userId: id,
            roleId: observerRole[0].id,
          })
          .onConflictDoNothing();
      }

      return new Response('User created successfully', { status: 200 });
    } catch (error) {
      console.error('Error creating user:', error);
      return new Response('Database error', { status: 500 });
    }
  }

  if (eventType === 'user.deleted') {
    try {
      await db
        .delete(usersTable)
        .where(eq(usersTable.id, String(event.data.id)));
    } catch (error) {
      console.error('Error deleting user from database:', error);
      return new Response('Database error', { status: 500 });
    }
  }

  return new Response('Webhook processed successfully', { status: 200 });
}