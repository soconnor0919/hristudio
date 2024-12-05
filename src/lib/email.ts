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
  const roleDisplay = role
    .split('_')
    .map(word => word.charAt(0).toUpperCase() + word.slice(1).toLowerCase())
    .join(' ');

  const html = `
    <!DOCTYPE html>
    <html>
      <head>
        <meta name="viewport" content="width=device-width, initial-scale=1.0">
        <meta http-equiv="Content-Type" content="text/html; charset=UTF-8">
        <title>HRIStudio Invitation</title>
        <style>
          @media only screen and (max-width: 620px) {
            .content {
              padding: 24px !important;
            }
          }
        </style>
      </head>
      <body style="
        background-color: #f3f4f6;
        font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, Helvetica, Arial, sans-serif;
        -webkit-font-smoothing: antialiased;
        font-size: 16px;
        line-height: 1.5;
        margin: 0;
        padding: 0;
      ">
        <table width="100%" cellpadding="0" cellspacing="0" role="presentation" style="margin: 0; padding: 20px 0;">
          <tr>
            <td align="center">
              <table width="100%" cellpadding="0" cellspacing="0" role="presentation" style="
                max-width: 600px;
                margin: 0 auto;
                padding: 0;
              ">
                <!-- Logo -->
                <tr>
                  <td align="center" style="padding: 24px;">
                    <div style="display: flex; align-items: center; justify-content: center; margin-bottom: 8px;">
                      <svg
                        width="24"
                        height="24"
                        viewBox="0 0 24 24"
                        fill="none"
                        stroke="#6b7280"
                        stroke-width="2"
                        stroke-linecap="round"
                        stroke-linejoin="round"
                        style="margin-right: 4px;"
                      >
                        <path d="M12 8V4H8" />
                        <rect width="16" height="12" x="4" y="8" rx="2" />
                        <path d="M2 14h2" />
                        <path d="M20 14h2" />
                        <path d="M15 13v2" />
                        <path d="M9 13v2" />
                      </svg>
                      <h1 style="
                        color: #111827;
                        font-size: 24px;
                        font-weight: bold;
                        margin: 0;
                        display: flex;
                      ">
                        <span style="font-weight: 800;">HRI</span>
                        <span style="font-weight: 400;">Studio</span>
                      </h1>
                    </div>
                    <p style="
                      color: #6b7280;
                      font-size: 14px;
                      margin: 8px 0 0;
                    ">A platform for managing human-robot interaction studies</p>
                  </td>
                </tr>
                <!-- Main Content -->
                <tr>
                  <td class="content" style="
                    background-color: #ffffff;
                    padding: 32px;
                    border-radius: 8px;
                    box-shadow: 0 1px 3px 0 rgba(0, 0, 0, 0.1), 0 1px 2px 0 rgba(0, 0, 0, 0.06);
                  ">
                    <h2 style="
                      color: #111827;
                      font-size: 20px;
                      font-weight: 600;
                      margin: 0 0 16px;
                    ">You've been invited to join a research study</h2>
                    
                    <p style="margin: 16px 0; color: #374151;">
                      ${inviterName} has invited you to join <strong>"${studyTitle}"</strong> as a ${roleDisplay}.
                    </p>

                    <p style="margin: 16px 0; color: #374151;">
                      HRIStudio helps research teams manage human-robot interaction studies and conduct Wizard-of-Oz experiments efficiently.
                    </p>

                    <!-- Button -->
                    <table width="100%" cellpadding="0" cellspacing="0" role="presentation">
                      <tr>
                        <td align="center" style="padding: 24px 0;">
                          <a href="${inviteUrl}" style="
                            background-color: #2563eb;
                            border-radius: 6px;
                            color: #ffffff;
                            display: inline-block;
                            font-size: 16px;
                            font-weight: 500;
                            line-height: 1;
                            padding: 12px 24px;
                            text-decoration: none;
                          ">Accept Invitation</a>
                        </td>
                      </tr>
                    </table>

                    <p style="
                      margin: 16px 0 0;
                      padding-top: 16px;
                      border-top: 1px solid #e5e7eb;
                      color: #6b7280;
                      font-size: 14px;
                    ">
                      If you're having trouble with the button above, copy and paste the URL below into your web browser:
                    </p>
                    <p style="
                      margin: 8px 0;
                      color: #6b7280;
                      font-size: 14px;
                      word-break: break-all;
                    ">
                      ${inviteUrl}
                    </p>

                    <p style="
                      margin: 24px 0 0;
                      color: #6b7280;
                      font-size: 14px;
                      font-style: italic;
                    ">
                      This invitation will expire in 7 days.
                    </p>
                  </td>
                </tr>
                <!-- Footer -->
                <tr>
                  <td style="padding: 24px; text-align: center;">
                    <p style="
                      color: #6b7280;
                      font-size: 14px;
                      margin: 0;
                    ">
                      This is an automated message from HRIStudio. Please do not reply to this email.
                    </p>
                  </td>
                </tr>
              </table>
            </td>
          </tr>
        </table>
      </body>
    </html>
  `;

  const text = `
You've been invited to join HRIStudio

${inviterName} has invited you to join "${studyTitle}" as a ${roleDisplay}.

HRIStudio helps research teams manage human-robot interaction studies and conduct Wizard-of-Oz experiments efficiently.

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