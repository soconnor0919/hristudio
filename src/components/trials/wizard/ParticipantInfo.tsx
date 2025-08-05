"use client";

import {
    Briefcase, Clock, GraduationCap, Info, Mail, Shield, User
} from "lucide-react";
import { Avatar, AvatarFallback } from "~/components/ui/avatar";
import { Card, CardContent, CardHeader, CardTitle } from "~/components/ui/card";

interface ParticipantInfoProps {
  participant: {
    id: string;
    participantCode: string;
    email: string | null;
    name: string | null;
    demographics: any;
  };
}

export function ParticipantInfo({ participant }: ParticipantInfoProps) {
  const demographics = participant.demographics || {};

  // Extract common demographic fields
  const age = demographics.age;
  const gender = demographics.gender;
  const occupation = demographics.occupation;
  const education = demographics.education;
  const language = demographics.primaryLanguage || demographics.language;
  const location = demographics.location || demographics.city;
  const experience = demographics.robotExperience || demographics.experience;

  // Get participant initials for avatar
  const getInitials = () => {
    if (participant.name) {
      const nameParts = participant.name.split(" ");
      return nameParts.map((part) => part.charAt(0).toUpperCase()).join("");
    }
    return participant.participantCode.substring(0, 2).toUpperCase();
  };

  const formatDemographicValue = (key: string, value: any) => {
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

    return String(value);
  };

  return (
    <div className="space-y-4">
      <div className="flex items-center space-x-2">
        <User className="h-4 w-4 text-slate-600" />
        <h3 className="font-medium text-slate-900">Participant</h3>
      </div>

      {/* Basic Info Card */}
      <Card className="shadow-sm">
        <CardContent className="p-4">
          <div className="flex items-start space-x-3">
            <Avatar className="h-10 w-10">
              <AvatarFallback className="bg-blue-100 font-medium text-blue-600">
                {getInitials()}
              </AvatarFallback>
            </Avatar>
            <div className="min-w-0 flex-1">
              <div className="truncate font-medium text-slate-900">
                {participant.name || "Anonymous"}
              </div>
              <div className="text-sm text-slate-600">
                ID: {participant.participantCode}
              </div>
              {participant.email && (
                <div className="mt-1 flex items-center space-x-1 text-xs text-slate-500">
                  <Mail className="h-3 w-3" />
                  <span className="truncate">{participant.email}</span>
                </div>
              )}
            </div>
          </div>
        </CardContent>
      </Card>

      {/* Quick Demographics */}
      {(age || gender || language) && (
        <Card className="shadow-sm">
          <CardContent className="p-4">
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
          </CardContent>
        </Card>
      )}

      {/* Background Info */}
      {(occupation || education || experience) && (
        <Card className="shadow-sm">
          <CardHeader className="pb-2">
            <CardTitle className="flex items-center space-x-1 text-sm font-medium text-slate-700">
              <Info className="h-3 w-3" />
              <span>Background</span>
            </CardTitle>
          </CardHeader>
          <CardContent className="space-y-2 pt-0">
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
          </CardContent>
        </Card>
      )}

      {/* Additional Demographics */}
      {Object.keys(demographics).length > 0 && (
        <Card className="shadow-sm">
          <CardHeader className="pb-2">
            <CardTitle className="text-sm font-medium text-slate-700">
              Additional Info
            </CardTitle>
          </CardHeader>
          <CardContent className="pt-0">
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
          </CardContent>
        </Card>
      )}

      {/* Consent Status */}
      <Card className="border-green-200 bg-green-50 shadow-sm">
        <CardContent className="p-3">
          <div className="flex items-center space-x-2">
            <div className="h-2 w-2 rounded-full bg-green-500"></div>
            <span className="text-sm font-medium text-green-800">
              Consent Verified
            </span>
          </div>
          <div className="mt-1 text-xs text-green-600">
            Participant has provided informed consent
          </div>
        </CardContent>
      </Card>

      {/* Session Info */}
      <div className="space-y-1 text-xs text-slate-500">
        <div className="flex items-center space-x-1">
          <Clock className="h-3 w-3" />
          <span>Session started: {new Date().toLocaleTimeString()}</span>
        </div>
      </div>
    </div>
  );
}
