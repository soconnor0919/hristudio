// Participant type definitions for HRIStudio

export interface ParticipantDemographics {
  age?: number;
  gender?: string;
  grade?: number;
  background?: string;
  education?: string;
  occupation?: string;
  experience?: string;
  notes?: string;
  [key: string]: string | number | boolean | undefined;
}

export interface ParticipantWithStudy {
  id: string;
  studyId: string;
  participantCode: string;
  email: string | null;
  name: string | null;
  demographics: ParticipantDemographics | null;
  consentGiven: boolean;
  consentDate: Date | null;
  notes: string | null;
  createdAt: Date;
  updatedAt: Date;
  study: {
    id: string;
    name: string;
    institution: string | null;
  } | null;
}

export interface CreateParticipantData {
  studyId: string;
  participantCode: string;
  email?: string;
  name?: string;
  demographics?: ParticipantDemographics;
  consentGiven?: boolean;
  consentDate?: Date;
  notes?: string;
}

export interface UpdateParticipantData {
  participantCode?: string;
  email?: string;
  name?: string;
  demographics?: ParticipantDemographics;
  consentGiven?: boolean;
  consentDate?: Date;
  notes?: string;
}

export interface ParticipantListItem {
  id: string;
  studyId: string;
  participantCode: string;
  name: string | null;
  consentGiven: boolean;
  consentDate: Date | null;
  createdAt: Date;
  studyName: string;
}

export interface ParticipantTrialSummary {
  id: string;
  experimentName: string;
  status: "scheduled" | "in_progress" | "completed" | "aborted";
  scheduledAt: Date | null;
  startedAt: Date | null;
  completedAt: Date | null;
  duration: number | null;
}

export interface ParticipantDetailData extends ParticipantWithStudy {
  trials: ParticipantTrialSummary[];
  upcomingTrials: ParticipantTrialSummary[];
  completedTrials: ParticipantTrialSummary[];
}
