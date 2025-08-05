"use client";

import { format, formatDistanceToNow } from "date-fns";
import {
    AlertCircle,
    CheckCircle,
    Clock, Download, Eye, MoreHorizontal, Plus,
    Search, Shield, Target, Trash2, Upload, Users, UserX
} from "lucide-react";
import { useRouter } from "next/navigation";
import { useCallback, useState } from "react";
import { Alert, AlertDescription } from "~/components/ui/alert";
import { Badge } from "~/components/ui/badge";
import { Button } from "~/components/ui/button";
import { Card, CardContent, CardHeader, CardTitle } from "~/components/ui/card";
import {
    Dialog,
    DialogContent,
    DialogDescription,
    DialogFooter,
    DialogHeader,
    DialogTitle
} from "~/components/ui/dialog";
import {
    DropdownMenu,
    DropdownMenuContent,
    DropdownMenuItem,
    DropdownMenuLabel,
    DropdownMenuSeparator,
    DropdownMenuTrigger
} from "~/components/ui/dropdown-menu";
import { Input } from "~/components/ui/input";
import { Label } from "~/components/ui/label";
import {
    Select,
    SelectContent,
    SelectItem,
    SelectTrigger,
    SelectValue
} from "~/components/ui/select";
import {
    Table,
    TableBody,
    TableCell,
    TableHead,
    TableHeader,
    TableRow
} from "~/components/ui/table";
import { Textarea } from "~/components/ui/textarea";
import { api } from "~/trpc/react";

interface Participant {
  id: string;
  participantCode: string;
  email: string | null;
  name: string | null;
  demographics: any;
  consentGiven: boolean;
  consentDate: Date | null;
  notes: string | null;
  createdAt: Date;
  updatedAt: Date;
  studyId: string;
  _count?: {
    trials: number;
  };
}

export function ParticipantsView() {
  const router = useRouter();
  const [searchQuery, setSearchQuery] = useState("");
  const [studyFilter, setStudyFilter] = useState<string>("all");
  const [consentFilter, setConsentFilter] = useState<string>("all");
  const [sortBy, setSortBy] = useState<string>("createdAt");
  const [sortOrder, setSortOrder] = useState<"asc" | "desc">("desc");
  const [showNewParticipantDialog, setShowNewParticipantDialog] =
    useState(false);
  const [showConsentDialog, setShowConsentDialog] = useState(false);
  const [selectedParticipant, setSelectedParticipant] =
    useState<Participant | null>(null);
  const [newParticipant, setNewParticipant] = useState({
    participantCode: "",
    email: "",
    name: "",
    studyId: "",
    demographics: {},
    notes: "",
  });

  // Get current user's studies
  const { data: userStudies } = api.studies.list.useQuery({
    memberOnly: true,
    limit: 100,
  });

  // Get participants with filtering
  const {
    data: participantsData,
    isLoading: participantsLoading,
    refetch,
  } = api.participants.list.useQuery(
    {
      studyId:
        studyFilter === "all"
          ? userStudies?.studies?.[0]?.id || ""
          : studyFilter,
      search: searchQuery || undefined,
      limit: 100,


    },
    {
      enabled: !!userStudies?.studies?.length,
    },
  );

  // Mutations
  const createParticipantMutation = api.participants.create.useMutation({
    onSuccess: () => {
      refetch();
      setShowNewParticipantDialog(false);
      resetNewParticipantForm();
    },
  });

  const updateConsentMutation = api.participants.update.useMutation({
    onSuccess: () => {
      refetch();
      setShowConsentDialog(false);
      setSelectedParticipant(null);
    },
  });

  const deleteParticipantMutation = api.participants.delete.useMutation({
    onSuccess: () => {
      refetch();
    },
  });

  const resetNewParticipantForm = () => {
    setNewParticipant({
      participantCode: "",
      email: "",
      name: "",
      studyId: "",
      demographics: {},
      notes: "",
    });
  };

  const handleCreateParticipant = useCallback(async () => {
    if (!newParticipant.participantCode || !newParticipant.studyId) return;

    try {
      await createParticipantMutation.mutateAsync({
        participantCode: newParticipant.participantCode,
        studyId: newParticipant.studyId,
        email: newParticipant.email || undefined,
        name: newParticipant.name || undefined,
        demographics: newParticipant.demographics,

      });
    } catch (_error) {
      console.error("Failed to create participant:", _error);
    }
  }, [newParticipant, createParticipantMutation]);

  const handleUpdateConsent = useCallback(
    async (consentGiven: boolean) => {
      if (!selectedParticipant) return;

      try {
        await updateConsentMutation.mutateAsync({
          id: selectedParticipant.id,

        });
      } catch (_error) {
        console.error("Failed to update consent:", _error);
      }
    },
    [selectedParticipant, updateConsentMutation],
  );

  const handleDeleteParticipant = useCallback(
    async (participantId: string) => {
      if (
        !confirm(
          "Are you sure you want to delete this participant? This action cannot be undone.",
        )
      ) {
        return;
      }

      try {
        await deleteParticipantMutation.mutateAsync({ id: participantId });
      } catch (_error) {
        console.error("Failed to delete participant:", _error);
      }
    },
    [deleteParticipantMutation],
  );

  const getConsentStatusBadge = (participant: Participant) => {
    if (participant.consentGiven) {
      return (
        <Badge className="bg-green-100 text-green-800">
          <CheckCircle className="mr-1 h-3 w-3" />
          Consented
        </Badge>
      );
    } else {
      return (
        <Badge className="bg-red-100 text-red-800">
          <UserX className="mr-1 h-3 w-3" />
          Pending
        </Badge>
      );
    }
  };

  const getTrialsBadge = (trialCount: number) => {
    if (trialCount === 0) {
      return <Badge variant="outline">No trials</Badge>;
    } else if (trialCount === 1) {
      return <Badge className="bg-blue-100 text-blue-800">1 trial</Badge>;
    } else {
      return (
        <Badge className="bg-blue-100 text-blue-800">{trialCount} trials</Badge>
      );
    }
  };

  const filteredParticipants =
    participantsData?.participants?.filter((participant) => {
      if (consentFilter === "consented" && !participant.consentGiven)
        return false;
      if (consentFilter === "pending" && participant.consentGiven) return false;
      return true;
    }) || [];

  return (
    <div className="space-y-6">
      {/* Header Actions */}
      <Card>
        <CardHeader>
          <div className="flex items-center justify-between">
            <div>
              <CardTitle>Participant Management</CardTitle>
              <p className="mt-1 text-sm text-slate-600">
                Manage participant registration, consent, and trial assignments
              </p>
            </div>
            <div className="flex space-x-2">
              <Button variant="outline" size="sm">
                <Upload className="mr-2 h-4 w-4" />
                Import
              </Button>
              <Button variant="outline" size="sm">
                <Download className="mr-2 h-4 w-4" />
                Export
              </Button>
              <Button
                onClick={() => setShowNewParticipantDialog(true)}
                size="sm"
              >
                <Plus className="mr-2 h-4 w-4" />
                Add Participant
              </Button>
            </div>
          </div>
        </CardHeader>
      </Card>

      {/* Filters and Search */}
      <Card>
        <CardContent className="pt-6">
          <div className="flex flex-col space-y-4 md:flex-row md:space-y-0 md:space-x-4">
            <div className="flex-1">
              <Label htmlFor="search" className="sr-only">
                Search participants
              </Label>
              <div className="relative">
                <Search className="absolute top-1/2 left-3 h-4 w-4 -translate-y-1/2 text-slate-400" />
                <Input
                  id="search"
                  placeholder="Search by code, name, or email..."
                  value={searchQuery}
                  onChange={(e) => setSearchQuery(e.target.value)}
                  className="pl-10"
                />
              </div>
            </div>
            <Select value={studyFilter} onValueChange={setStudyFilter}>
              <SelectTrigger className="w-48">
                <SelectValue placeholder="Filter by study" />
              </SelectTrigger>
              <SelectContent>
                <SelectItem value="all">All Studies</SelectItem>
                {userStudies?.studies?.map((study: any) => (
                  <SelectItem key={study.id} value={study.id}>
                    {study.name}
                  </SelectItem>
                ))}
              </SelectContent>
            </Select>
            <Select value={consentFilter} onValueChange={setConsentFilter}>
              <SelectTrigger className="w-40">
                <SelectValue placeholder="Consent status" />
              </SelectTrigger>
              <SelectContent>
                <SelectItem value="all">All Status</SelectItem>
                <SelectItem value="consented">Consented</SelectItem>
                <SelectItem value="pending">Pending</SelectItem>
              </SelectContent>
            </Select>
            <Select
              value={`${sortBy}-${sortOrder}`}
              onValueChange={(value) => {
                const [field, order] = value.split("-");
                setSortBy(field || "createdAt");
                setSortOrder(order as "asc" | "desc");
              }}
            >
              <SelectTrigger className="w-40">
                <SelectValue placeholder="Sort by" />
              </SelectTrigger>
              <SelectContent>
                <SelectItem value="createdAt-desc">Newest first</SelectItem>
                <SelectItem value="createdAt-asc">Oldest first</SelectItem>
                <SelectItem value="participantCode-asc">Code A-Z</SelectItem>
                <SelectItem value="participantCode-desc">Code Z-A</SelectItem>
                <SelectItem value="name-asc">Name A-Z</SelectItem>
                <SelectItem value="name-desc">Name Z-A</SelectItem>
              </SelectContent>
            </Select>
          </div>
        </CardContent>
      </Card>

      {/* Statistics */}
      <div className="grid grid-cols-1 gap-4 md:grid-cols-4">
        <Card>
          <CardContent className="pt-6">
            <div className="flex items-center space-x-2">
              <Users className="h-8 w-8 text-blue-600" />
              <div>
                <p className="text-2xl font-bold">
                  {participantsData?.pagination?.total || 0}
                </p>
                <p className="text-xs text-slate-600">Total Participants</p>
              </div>
            </div>
          </CardContent>
        </Card>
        <Card>
          <CardContent className="pt-6">
            <div className="flex items-center space-x-2">
              <CheckCircle className="h-8 w-8 text-green-600" />
              <div>
                <p className="text-2xl font-bold">
                  {filteredParticipants.filter((p) => p.consentGiven).length}
                </p>
                <p className="text-xs text-slate-600">Consented</p>
              </div>
            </div>
          </CardContent>
        </Card>
        <Card>
          <CardContent className="pt-6">
            <div className="flex items-center space-x-2">
              <Clock className="h-8 w-8 text-yellow-600" />
              <div>
                <p className="text-2xl font-bold">
                  {filteredParticipants.filter((p) => !p.consentGiven).length}
                </p>
                <p className="text-xs text-slate-600">Pending Consent</p>
              </div>
            </div>
          </CardContent>
        </Card>
        <Card>
          <CardContent className="pt-6">
            <div className="flex items-center space-x-2">
              <Target className="h-8 w-8 text-purple-600" />
              <div>
                <p className="text-2xl font-bold">
                  {filteredParticipants.reduce(
                    (sum, p) => sum + (p.trialCount || 0),
                    0,
                  )}
                </p>
                <p className="text-xs text-slate-600">Total Trials</p>
              </div>
            </div>
          </CardContent>
        </Card>
      </div>

      {/* Participants Table */}
      <Card>
        <CardContent className="p-0">
          {participantsLoading ? (
            <div className="flex items-center justify-center py-12">
              <div className="text-center">
                <Users className="mx-auto h-8 w-8 animate-pulse text-slate-400" />
                <p className="mt-2 text-sm text-slate-500">
                  Loading participants...
                </p>
              </div>
            </div>
          ) : filteredParticipants.length === 0 ? (
            <div className="flex items-center justify-center py-12">
              <div className="text-center">
                <Users className="mx-auto h-8 w-8 text-slate-300" />
                <p className="mt-2 text-sm text-slate-500">
                  No participants found
                </p>
                <p className="text-xs text-slate-400">
                  {searchQuery ||
                  studyFilter !== "all" ||
                  consentFilter !== "all"
                    ? "Try adjusting your filters"
                    : "Add your first participant to get started"}
                </p>
              </div>
            </div>
          ) : (
            <Table>
              <TableHeader>
                <TableRow>
                  <TableHead>Participant</TableHead>
                  <TableHead>Study</TableHead>
                  <TableHead>Consent Status</TableHead>
                  <TableHead>Trials</TableHead>
                  <TableHead>Registered</TableHead>
                  <TableHead className="w-12"></TableHead>
                </TableRow>
              </TableHeader>
              <TableBody>
                {filteredParticipants.map((participant) => (
                  <TableRow key={participant.id}>
                    <TableCell>
                      <div className="flex items-center space-x-3">
                        <div className="flex h-8 w-8 items-center justify-center rounded-full bg-blue-100">
                          <span className="text-sm font-medium text-blue-600">
                            {participant.participantCode
                              .slice(0, 2)
                              .toUpperCase()}
                          </span>
                        </div>
                        <div>
                          <p className="font-medium">
                            {participant.participantCode}
                          </p>
                          {participant.name && (
                            <p className="text-sm text-slate-600">
                              {participant.name}
                            </p>
                          )}
                          {participant.email && (
                            <p className="text-xs text-slate-500">
                              {participant.email}
                            </p>
                          )}
                        </div>
                      </div>
                    </TableCell>
                    <TableCell>
                      <div className="text-sm">
                        {userStudies?.studies?.find(
                          (s) => s.id === participant.studyId,
                        )?.name || "Unknown Study"}
                      </div>
                    </TableCell>
                    <TableCell>
                      {getConsentStatusBadge({...participant, demographics: null, notes: null})}
                      {participant.consentDate && (
                        <p className="mt-1 text-xs text-slate-500">
                          {format(
                            new Date(participant.consentDate),
                            "MMM d, yyyy",
                          )}
                        </p>
                      )}
                    </TableCell>
                    <TableCell>
                      {getTrialsBadge(participant.trialCount || 0)}
                    </TableCell>
                    <TableCell>
                      <div className="text-sm text-slate-600">
                        {formatDistanceToNow(new Date(participant.createdAt), {
                          addSuffix: true,
                        })}
                      </div>
                    </TableCell>
                    <TableCell>
                      <DropdownMenu>
                        <DropdownMenuTrigger asChild>
                          <Button variant="ghost" size="sm">
                            <MoreHorizontal className="h-4 w-4" />
                          </Button>
                        </DropdownMenuTrigger>
                        <DropdownMenuContent align="end">
                          <DropdownMenuLabel>Actions</DropdownMenuLabel>
                          <DropdownMenuItem
                            onClick={() =>
                              router.push(`/participants/${participant.id}`)
                            }
                          >
                            <Eye className="mr-2 h-4 w-4" />
                            View Details
                          </DropdownMenuItem>
                          <DropdownMenuItem
                            onClick={() => {
                              setSelectedParticipant({...participant, demographics: null, notes: null});
                              setShowConsentDialog(true);
                            }}
                          >
                            <Shield className="mr-2 h-4 w-4" />
                            Manage Consent
                          </DropdownMenuItem>
                          <DropdownMenuSeparator />
                          <DropdownMenuItem
                            onClick={() =>
                              handleDeleteParticipant(participant.id)
                            }
                            className="text-red-600"
                          >
                            <Trash2 className="mr-2 h-4 w-4" />
                            Delete
                          </DropdownMenuItem>
                        </DropdownMenuContent>
                      </DropdownMenu>
                    </TableCell>
                  </TableRow>
                ))}
              </TableBody>
            </Table>
          )}
        </CardContent>
      </Card>

      {/* New Participant Dialog */}
      <Dialog
        open={showNewParticipantDialog}
        onOpenChange={setShowNewParticipantDialog}
      >
        <DialogContent className="max-w-md">
          <DialogHeader>
            <DialogTitle>Add New Participant</DialogTitle>
            <DialogDescription>
              Register a new participant for study enrollment
            </DialogDescription>
          </DialogHeader>
          <div className="space-y-4">
            <div>
              <Label htmlFor="participantCode">Participant Code *</Label>
              <Input
                id="participantCode"
                value={newParticipant.participantCode}
                onChange={(e) =>
                  setNewParticipant((prev) => ({
                    ...prev,
                    participantCode: e.target.value,
                  }))
                }
                placeholder="P001, SUBJ_01, etc."
                className="mt-1"
              />
            </div>
            <div>
              <Label htmlFor="study">Study *</Label>
              <Select
                value={newParticipant.studyId}
                onValueChange={(value) =>
                  setNewParticipant((prev) => ({ ...prev, studyId: value }))
                }
              >
                <SelectTrigger className="mt-1">
                  <SelectValue placeholder="Select study..." />
                </SelectTrigger>
                <SelectContent>
                  {userStudies?.studies?.map((study) => (
                    <SelectItem key={study.id} value={study.id}>
                      {study.name}
                    </SelectItem>
                  ))}
                </SelectContent>
              </Select>
            </div>
            <div>
              <Label htmlFor="name">Name (optional)</Label>
              <Input
                id="name"
                value={newParticipant.name}
                onChange={(e) =>
                  setNewParticipant((prev) => ({
                    ...prev,
                    name: e.target.value,
                  }))
                }
                placeholder="Participant's name"
                className="mt-1"
              />
            </div>
            <div>
              <Label htmlFor="email">Email (optional)</Label>
              <Input
                id="email"
                type="email"
                value={newParticipant.email}
                onChange={(e) =>
                  setNewParticipant((prev) => ({
                    ...prev,
                    email: e.target.value,
                  }))
                }
                placeholder="participant@example.com"
                className="mt-1"
              />
            </div>
            <div>
              <Label htmlFor="notes">Notes (optional)</Label>
              <Textarea
                id="notes"
                value={newParticipant.notes}
                onChange={(e) =>
                  setNewParticipant((prev) => ({
                    ...prev,
                    notes: e.target.value,
                  }))
                }
                placeholder="Additional notes about this participant..."
                className="mt-1"
                rows={3}
              />
            </div>
          </div>
          <DialogFooter>
            <Button
              variant="outline"
              onClick={() => {
                setShowNewParticipantDialog(false);
                resetNewParticipantForm();
              }}
            >
              Cancel
            </Button>
            <Button
              onClick={handleCreateParticipant}
              disabled={
                !newParticipant.participantCode ||
                !newParticipant.studyId ||
                createParticipantMutation.isPending
              }
            >
              {createParticipantMutation.isPending
                ? "Creating..."
                : "Create Participant"}
            </Button>
          </DialogFooter>
        </DialogContent>
      </Dialog>

      {/* Consent Management Dialog */}
      <Dialog open={showConsentDialog} onOpenChange={setShowConsentDialog}>
        <DialogContent>
          <DialogHeader>
            <DialogTitle>Manage Consent</DialogTitle>
            <DialogDescription>
              Update consent status for {selectedParticipant?.participantCode}
            </DialogDescription>
          </DialogHeader>
          {selectedParticipant && (
            <div className="space-y-4">
              <div className="rounded-lg border bg-slate-50 p-4">
                <h4 className="font-medium">Current Status</h4>
                <div className="mt-2 flex items-center space-x-2">
                  {getConsentStatusBadge(selectedParticipant)}
                  {selectedParticipant.consentDate && (
                    <span className="text-sm text-slate-600">
                      on{" "}
                      {format(new Date(selectedParticipant.consentDate), "PPP")}
                    </span>
                  )}
                </div>
              </div>

              <Alert>
                <AlertCircle className="h-4 w-4" />
                <AlertDescription>
                  Updating consent status will be logged for audit purposes.
                  Ensure you have proper authorization before proceeding.
                </AlertDescription>
              </Alert>

              <div className="flex space-x-2">
                <Button
                  onClick={() => handleUpdateConsent(true)}
                  disabled={
                    selectedParticipant.consentGiven ||
                    updateConsentMutation.isPending
                  }
                  className="flex-1"
                >
                  <CheckCircle className="mr-2 h-4 w-4" />
                  Grant Consent
                </Button>
                <Button
                  variant="outline"
                  onClick={() => handleUpdateConsent(false)}
                  disabled={
                    !selectedParticipant.consentGiven ||
                    updateConsentMutation.isPending
                  }
                  className="flex-1"
                >
                  <UserX className="mr-2 h-4 w-4" />
                  Revoke Consent
                </Button>
              </div>
            </div>
          )}
          <DialogFooter>
            <Button
              variant="outline"
              onClick={() => {
                setShowConsentDialog(false);
                setSelectedParticipant(null);
              }}
            >
              Close
            </Button>
          </DialogFooter>
        </DialogContent>
      </Dialog>
    </div>
  );
}
