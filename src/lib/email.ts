import nodemailer from 'nodemailer';

// Create reusable transporter object using SMTP transport
const transporter = nodemailer.createTransport({
  service: 'iCloud',
  secure: false,
  auth: {
    user: 'soconnor0919@icloud.com',
    pass: 'uhlb-virv-qqpk-puwc',
  },
});

// Verify connection configuration
transporter.verify(function(error, success) {
  if (error) {
    console.log('SMTP Verification Error:', error);
  }
});

interface SendInvitationEmailParams {
  to: string;
  inviterName: string;
  studyTitle: string;
  role: string;
  token: string;
}

export async function sendInvitationEmail({
  to,
  inviterName,
  studyTitle,
  role,
  token,
}: SendInvitationEmailParams) {
  const inviteUrl = `${process.env.NEXT_PUBLIC_APP_URL}/invite/accept/${token}`;

  const html = `
    <h2>You've been invited to join HRIStudio</h2>
    <p>${inviterName} has invited you to join their study "${studyTitle}" as a ${role}.</p>
    <p>HRIStudio is a platform for managing human-robot interaction studies and Wizard-of-Oz experiments.</p>
    <p>Click the button below to accept the invitation and join the study:</p>
    <a href="${inviteUrl}" style="
      display: inline-block;
      background-color: #0070f3;
      color: white;
      padding: 12px 24px;
      text-decoration: none;
      border-radius: 6px;
      margin: 16px 0;
    ">Accept Invitation</a>
    <p>Or copy and paste this URL into your browser:</p>
    <p>${inviteUrl}</p>
    <p>This invitation will expire in 7 days.</p>
  `;

  const text = `
You've been invited to join HRIStudio

${inviterName} has invited you to join their study "${studyTitle}" as a ${role}.

HRIStudio is a platform for managing human-robot interaction studies and Wizard-of-Oz experiments.

To accept the invitation, visit this URL:
${inviteUrl}

This invitation will expire in 7 days.
  `;

  await transporter.sendMail({
    from: `"HRIStudio" <${process.env.SMTP_FROM_ADDRESS}>`,
    to,
    subject: `Invitation to join "${studyTitle}" on HRIStudio`,
    text,
    html,
  });
} 