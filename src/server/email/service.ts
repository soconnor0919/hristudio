import nodemailer from "nodemailer";
import { type EmailConfig, emailConfig } from "./config";

export class EmailService {
  private transporter: nodemailer.Transporter;
  private config: EmailConfig;

  constructor(config: EmailConfig = emailConfig) {
    this.config = config;
    this.transporter = nodemailer.createTransport(config.smtp);
  }

  async sendMail(options: {
    to: string;
    subject: string;
    text?: string;
    html?: string;
  }) {
    const { to, subject, text, html } = options;

    await this.transporter.sendMail({
      from: `"${this.config.from.name}" <${this.config.from.email}>`,
      to,
      subject,
      text,
      html,
    });
  }

  async sendStudyInvitation({
    to,
    studyTitle,
    role,
    inviteUrl,
  }: {
    to: string;
    studyTitle: string;
    role: string;
    inviteUrl: string;
  }) {
    const subject = `Invitation to join "${studyTitle}" as ${role}`;
    const html = `
      <div style="font-family: sans-serif; max-width: 600px; margin: 0 auto;">
        <h2>You've been invited!</h2>
        <p>You've been invited to join the study "${studyTitle}" as a ${role}.</p>
        <p style="margin: 24px 0;">
          <a href="${inviteUrl}" 
             style="background: #0091FF; color: white; padding: 12px 24px; 
                    text-decoration: none; border-radius: 6px; display: inline-block;">
            Accept Invitation
          </a>
        </p>
        <p style="color: #666; font-size: 14px;">
          If you can't click the button above, copy and paste this URL into your browser:<br>
          ${inviteUrl}
        </p>
        <p style="color: #666; font-size: 14px;">
          This invitation will expire in 7 days.
        </p>
      </div>
    `;

    const text = `
You've been invited!

You've been invited to join the study "${studyTitle}" as a ${role}.

To accept the invitation, visit this URL:
${inviteUrl}

This invitation will expire in 7 days.
    `.trim();

    await this.sendMail({ to, subject, html, text });
  }

  async verifyConnection() {
    try {
      await this.transporter.verify();
      return true;
    } catch (error) {
      console.error("Failed to verify email connection:", error);
      return false;
    }
  }
} 