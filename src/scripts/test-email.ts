import { config } from 'dotenv';
import { sendInvitationEmail } from '~/lib/email';

// Load environment variables from .env.local
config({ path: '.env.local' });

async function testEmail() {
  try {
    // Create a test invitation
    const invitationData = {
      to: 'soconnor0919@gmail.com',
      inviterName: 'Sean O\'Connor',
      studyTitle: 'Robot Navigation Study',
      role: 'Researcher',
      token: 'test-' + Math.random().toString(36).substring(2, 15),
    };

    console.log('Sending invitation with the following details:');
    console.log('To:', invitationData.to);
    console.log('Role:', invitationData.role);
    console.log('Study:', invitationData.studyTitle);
    console.log('Token:', invitationData.token);
    console.log('Invite URL:', `${process.env.NEXT_PUBLIC_APP_URL}/invite/accept/${invitationData.token}`);

    await sendInvitationEmail(invitationData);
    console.log('✅ Invitation email sent successfully!');
  } catch (error) {
    console.error('❌ Error sending invitation email:', error);
  }
}

testEmail();
