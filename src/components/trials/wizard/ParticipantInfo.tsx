"use client";

import { Briefcase, Clock, GraduationCap, Info, Shield } from "lucide-react";
import { Avatar, AvatarFallback } from "~/components/ui/avatar";

interface ParticipantInfoProps {
  participant: {
    id: string;
    participantCode: string;
    demographics: Record<string, unknown> | null;
  };
  trialStatus: "scheduled" | "in_progress" | "completed" | "aborted" | "failed";
}

export function ParticipantInfo({
  participant,
  trialStatus: _trialStatus,
}: ParticipantInfoProps) {
  const demographics = participant.demographics ?? {};

  // Extract common demographic fields
  const age = demographics.age as string | number | undefined;
  const gender = demographics.gender as string | undefined;
  const occupation = demographics.occupation as string | undefined;
  const education = demographics.education as string | undefined;
  const language =
    (demographics.primaryLanguage as string | undefined) ??
    (demographics.language as string | undefined);
  const experience =
    (demographics.robotExperience as string | undefined) ??
    (demographics.experience as string | undefined);

  // Get participant initials for avatar
  const getInitials = () => {
    return participant.participantCode.substring(0, 2).toUpperCase();
  };

  const formatDemographicValue = (key: string, value: unknown) => {
    if (value === null || value === undefined || value === "") return null;

    // Handle different data types
    if (typeof value === "boolean") {
      return value ? "Yes" : "No";
    }

    if (Array.isArray(value)) {
      return value.join(", ");
    }

    if (typeof value === "object") {
      return JSON.stringify(value);
    }

    return typeof value === "string" ? value : JSON.stringify(value);
  };

  return (
    <div className="space-y-4">
      {/* Basic Info */}
      <div className="rounded-lg border p-4">
        <div className="flex items-start space-x-3">
          <Avatar className="h-10 w-10">
            <AvatarFallback className="font-medium">
              {getInitials()}
            </AvatarFallback>
          </Avatar>
          <div className="min-w-0 flex-1">
            <div className="truncate font-medium text-slate-900">
              Participant {participant.participantCode}
            </div>
            <div className="text-sm text-slate-600">
              ID: {participant.participantCode}
            </div>
          </div>
        </div>
      </div>

      {/* Quick Demographics */}
      {(age ?? gender ?? language) && (
        <div className="rounded-lg border p-4">
          <div className="grid grid-cols-1 gap-2 text-sm">
            {age && (
              <div className="flex items-center justify-between">
                <span className="text-slate-600">Age:</span>
                <span className="font-medium">{age}</span>
              </div>
            )}
            {gender && (
              <div className="flex items-center justify-between">
                <span className="text-slate-600">Gender:</span>
                <span className="font-medium capitalize">{gender}</span>
              </div>
            )}
            {language && (
              <div className="flex items-center justify-between">
                <span className="text-slate-600">Language:</span>
                <span className="font-medium">{language}</span>
              </div>
            )}
          </div>
        </div>
      )}

      {/* Background Info */}
      {(occupation ?? education ?? experience) && (
        <div className="rounded-lg border p-4">
          <div className="mb-3 flex items-center space-x-1 text-sm font-medium text-slate-700">
            <Info className="h-3 w-3" />
            <span>Background</span>
          </div>
          <div className="space-y-2">
            {occupation && (
              <div className="flex items-start space-x-2 text-sm">
                <Briefcase className="mt-0.5 h-3 w-3 flex-shrink-0 text-slate-400" />
                <div>
                  <div className="text-slate-600">Occupation</div>
                  <div className="text-xs font-medium">{occupation}</div>
                </div>
              </div>
            )}
            {education && (
              <div className="flex items-start space-x-2 text-sm">
                <GraduationCap className="mt-0.5 h-3 w-3 flex-shrink-0 text-slate-400" />
                <div>
                  <div className="text-slate-600">Education</div>
                  <div className="text-xs font-medium">{education}</div>
                </div>
              </div>
            )}
            {experience && (
              <div className="flex items-start space-x-2 text-sm">
                <Shield className="mt-0.5 h-3 w-3 flex-shrink-0 text-slate-400" />
                <div>
                  <div className="text-slate-600">Robot Experience</div>
                  <div className="text-xs font-medium">{experience}</div>
                </div>
              </div>
            )}
          </div>
        </div>
      )}

      {/* Additional Demographics */}
      {Object.keys(demographics).length > 0 && (
        <div className="rounded-lg border p-4">
          <div className="mb-3 text-sm font-medium text-slate-700">
            Additional Info
          </div>
          <div>
            <div className="space-y-1">
              {Object.entries(demographics)
                .filter(
                  ([key, value]) =>
                    ![
                      "age",
                      "gender",
                      "occupation",
                      "education",
                      "language",
                      "primaryLanguage",
                      "robotExperience",
                      "experience",
                      "location",
                      "city",
                    ].includes(key) &&
                    value !== null &&
                    value !== undefined &&
                    value !== "",
                )
                .slice(0, 5) // Limit to 5 additional fields
                .map(([key, value]) => {
                  const formattedValue = formatDemographicValue(key, value);
                  if (!formattedValue) return null;

                  return (
                    <div
                      key={key}
                      className="flex items-center justify-between text-xs"
                    >
                      <span className="text-slate-600 capitalize">
                        {key
                          .replace(/([A-Z])/g, " $1")
                          .replace(/^./, (str) => str.toUpperCase())}
                        :
                      </span>
                      <span className="ml-2 max-w-[120px] truncate text-right font-medium">
                        {formattedValue}
                      </span>
                    </div>
                  );
                })}
            </div>
          </div>
        </div>
      )}

      {/* Consent Status */}
      <div className="rounded-lg border p-3">
        <div className="flex items-center space-x-2">
          <div className="h-2 w-2 rounded-full bg-green-500"></div>
          <span className="text-sm font-medium">Consent Verified</span>
        </div>
        <div className="text-muted-foreground mt-1 text-xs">
          Participant has provided informed consent
        </div>
      </div>

      {/* Session Info */}
      <div className="space-y-1 text-xs text-slate-500">
        <div className="flex items-center space-x-1">
          <Clock className="h-3 w-3" />
          <span>Session active</span>
        </div>
      </div>
    </div>
  );
}
