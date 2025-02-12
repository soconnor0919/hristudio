"use client";

import { useRouter } from "next/navigation";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "~/components/ui/card";
import { Button } from "~/components/ui/button";
import {
  Table,
  TableBody,
  TableCell,
  TableHead,
  TableHeader,
  TableRow,
} from "~/components/ui/table";
import { Badge } from "~/components/ui/badge";
import { api } from "~/trpc/react";
import { Plus as PlusIcon, Eye, EyeOff } from "lucide-react";
import { ROLES } from "~/lib/permissions/constants";
import { Switch } from "~/components/ui/switch";
import { Label } from "~/components/ui/label";
import { useState } from "react";

interface StudyParticipantsProps {
  studyId: number;
  role: string;
}

export function StudyParticipants({ studyId, role }: StudyParticipantsProps) {
  const router = useRouter();
  const { data: participants, isLoading } = api.participant.getByStudyId.useQuery({ studyId });
  const [showIdentifiable, setShowIdentifiable] = useState(false);

  const canViewIdentifiableInfo = [ROLES.OWNER, ROLES.ADMIN, ROLES.PRINCIPAL_INVESTIGATOR]
    .map(r => r.toLowerCase())
    .includes(role.toLowerCase());

  const canManageParticipants = [ROLES.OWNER, ROLES.ADMIN, ROLES.PRINCIPAL_INVESTIGATOR]
    .map(r => r.toLowerCase())
    .includes(role.toLowerCase());

  if (isLoading) {
    return <div>Loading...</div>;
  }

  return (
    <Card>
      <CardHeader>
        <div className="flex items-center justify-between">
          <div>
            <CardTitle>Study Participants</CardTitle>
            {!canViewIdentifiableInfo ? (
              <CardDescription className="text-yellow-600">
                Personal information is redacted based on your role.
              </CardDescription>
            ) : (
              <CardDescription>
                {showIdentifiable 
                  ? "Showing personal information."
                  : "Personal information is hidden."}
              </CardDescription>
            )}
          </div>
          <div className="flex items-center gap-4">
            {canViewIdentifiableInfo && (
              <div className="flex items-center gap-2 border rounded-lg p-2 bg-muted/50">
                <div className="flex items-center gap-2">
                  <Switch
                    id="show-identifiable"
                    checked={showIdentifiable}
                    onCheckedChange={setShowIdentifiable}
                  />
                  <Label 
                    htmlFor="show-identifiable" 
                    className="text-sm font-medium flex items-center gap-2 cursor-pointer select-none"
                  >
                    {showIdentifiable ? (
                      <>
                        <Eye className="h-4 w-4" />
                        Personal Info Visible
                      </>
                    ) : (
                      <>
                        <EyeOff className="h-4 w-4" />
                        Personal Info Hidden  
                      </>
                    )}
                  </Label>
                </div>
              </div>
            )}
            {canManageParticipants && (
              <Button
                size="sm"
                onClick={() => router.push(`/dashboard/studies/${studyId}/participants/new`)}
              >
                <PlusIcon className="h-4 w-4 mr-2" />
                Add Participant
              </Button>
            )}
          </div>
        </div>
      </CardHeader>
      <CardContent>
        {!participants || participants.length === 0 ? (
          <div className="text-center py-6 text-muted-foreground">
            No participants have been added to this study yet.
          </div>
        ) : (
          <Table>
            <TableHeader>
              <TableRow>
                <TableHead>ID</TableHead>
                <TableHead>Status</TableHead>
                {(canViewIdentifiableInfo && showIdentifiable) && (
                  <>
                    <TableHead>Name</TableHead>
                    <TableHead>Email</TableHead>
                  </>
                )}
                <TableHead>Notes</TableHead>
              </TableRow>
            </TableHeader>
            <TableBody>
              {participants.map((participant) => (
                <TableRow
                  key={participant.id}
                  className="cursor-pointer hover:bg-muted/50"
                  onClick={() =>
                    router.push(`/dashboard/studies/${studyId}/participants/${participant.id}`)
                  }
                >
                  <TableCell>{participant.identifier || "—"}</TableCell>
                  <TableCell>
                    <Badge
                      variant={
                        participant.status === "active"
                          ? "default"
                          : participant.status === "completed"
                          ? "secondary"
                          : "outline"
                      }
                    >
                      {participant.status}
                    </Badge>
                  </TableCell>
                  {(canViewIdentifiableInfo && showIdentifiable) && (
                    <>
                      <TableCell>
                        {participant.firstName && participant.lastName
                          ? `${participant.firstName} ${participant.lastName}`
                          : "—"}
                      </TableCell>
                      <TableCell>{participant.email || "—"}</TableCell>
                    </>
                  )}
                  <TableCell className="max-w-[200px] truncate">
                    {participant.notes || "—"}
                  </TableCell>
                </TableRow>
              ))}
            </TableBody>
          </Table>
        )}
      </CardContent>
    </Card>
  );
} 