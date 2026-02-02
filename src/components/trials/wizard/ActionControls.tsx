"use client";

import {
  AlertTriangle,
  Camera,
  Clock,
  Hand,
  HelpCircle,
  Lightbulb,
  MessageSquare,
  Pause,
  RotateCcw,
  Target,
  Video,
  VideoOff,
  Volume2,
  VolumeX,
  Zap,
} from "lucide-react";
import { useState } from "react";
import { Button } from "~/components/ui/button";
import { Card, CardContent, CardHeader, CardTitle } from "~/components/ui/card";
import {
  Dialog,
  DialogContent,
  DialogDescription,
  DialogFooter,
  DialogHeader,
  DialogTitle,
} from "~/components/ui/dialog";
import { Label } from "~/components/ui/label";
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from "~/components/ui/select";
import { Textarea } from "~/components/ui/textarea";

interface ActionControlsProps {
  trialId: string;
  currentStep: {
    id: string;
    name: string;
    type:
      | "wizard_action"
      | "robot_action"
      | "parallel_steps"
      | "conditional_branch";
    description?: string;
    parameters?: Record<string, unknown>;
    duration?: number;
  } | null;
  onActionComplete: (
    actionId: string,
    actionData: Record<string, unknown>,
  ) => void;
  isConnected: boolean;
}

interface QuickAction {
  id: string;
  label: string;
  icon: React.ComponentType<{ className?: string }>;
  type: "primary" | "secondary" | "emergency";
  action: string;
  description: string;
  requiresConfirmation?: boolean;
}

export function ActionControls({
  trialId: _trialId,
  currentStep,
  onActionComplete,
  isConnected: _isConnected,
}: ActionControlsProps) {
  const [isRecording, setIsRecording] = useState(false);
  const [isVideoOn, setIsVideoOn] = useState(true);
  const [isAudioOn, setIsAudioOn] = useState(true);
  const [isCommunicationOpen, setIsCommunicationOpen] = useState(false);
  const [interventionNote, setInterventionNote] = useState("");
  const [selectedEmergencyAction, setSelectedEmergencyAction] = useState("");
  const [showEmergencyDialog, setShowEmergencyDialog] = useState(false);

  // Quick action definitions
  const quickActions: QuickAction[] = [
    {
      id: "manual_intervention",
      label: "Manual Intervention",
      icon: Hand,
      type: "primary",
      action: "manual_intervention",
      description: "Take manual control of the interaction",
    },
    {
      id: "provide_hint",
      label: "Provide Hint",
      icon: Lightbulb,
      type: "primary",
      action: "provide_hint",
      description: "Give a helpful hint to the participant",
    },
    {
      id: "clarification",
      label: "Clarification",
      icon: HelpCircle,
      type: "primary",
      action: "clarification",
      description: "Provide clarification or explanation",
    },
    {
      id: "pause_interaction",
      label: "Pause",
      icon: Pause,
      type: "secondary",
      action: "pause_interaction",
      description: "Temporarily pause the interaction",
    },
    {
      id: "reset_step",
      label: "Reset Step",
      icon: RotateCcw,
      type: "secondary",
      action: "reset_step",
      description: "Reset the current step",
    },
    {
      id: "emergency_stop",
      label: "Emergency Stop",
      icon: AlertTriangle,
      type: "emergency",
      action: "emergency_stop",
      description: "Emergency stop all robot actions",
      requiresConfirmation: true,
    },
  ];

  const emergencyActions = [
    { value: "stop_robot", label: "Stop Robot Movement" },
    { value: "safe_position", label: "Move to Safe Position" },
    { value: "disable_motors", label: "Disable All Motors" },
    { value: "cut_power", label: "Emergency Power Cut" },
  ];

  const handleQuickAction = (action: QuickAction) => {
    if (action.requiresConfirmation) {
      setShowEmergencyDialog(true);
      return;
    }

    onActionComplete(action.id, {
      action_type: action.action,
      notes: action.description,
      timestamp: new Date().toISOString(),
    });
  };

  const handleEmergencyAction = () => {
    if (!selectedEmergencyAction) return;

    onActionComplete("emergency_action", {
      emergency_type: selectedEmergencyAction,
      notes: interventionNote || "Emergency action executed",
      timestamp: new Date().toISOString(),
    });

    setShowEmergencyDialog(false);
    setSelectedEmergencyAction("");
    setInterventionNote("");
  };

  const handleInterventionSubmit = () => {
    if (!interventionNote.trim()) return;

    onActionComplete("wizard_intervention", {
      intervention_type: "note",
      content: interventionNote,
      timestamp: new Date().toISOString(),
    });

    setInterventionNote("");
    setIsCommunicationOpen(false);
  };

  const toggleRecording = () => {
    const newState = !isRecording;
    setIsRecording(newState);

    onActionComplete("recording_control", {
      action: newState ? "start_recording" : "stop_recording",
      timestamp: new Date().toISOString(),
    });
  };

  const toggleVideo = () => {
    const newState = !isVideoOn;
    setIsVideoOn(newState);

    onActionComplete("video_control", {
      action: newState ? "video_on" : "video_off",
      timestamp: new Date().toISOString(),
    });
  };

  const toggleAudio = () => {
    const newState = !isAudioOn;
    setIsAudioOn(newState);

    onActionComplete("audio_control", {
      action: newState ? "audio_on" : "audio_off",
      timestamp: new Date().toISOString(),
    });
  };

  return (
    <div className="space-y-6">
      {/* Media Controls */}
      <Card>
        <CardHeader>
          <CardTitle className="flex items-center space-x-2">
            <Camera className="h-5 w-5" />
            <span>Media Controls</span>
          </CardTitle>
        </CardHeader>
        <CardContent>
          <div className="grid grid-cols-2 gap-3">
            <Button
              variant={isRecording ? "destructive" : "outline"}
              onClick={toggleRecording}
              className="flex items-center space-x-2"
            >
              <div
                className={`h-2 w-2 rounded-full ${isRecording ? "animate-pulse" : ""}`}
              ></div>
              <span>{isRecording ? "Stop Recording" : "Start Recording"}</span>
            </Button>

            <Button
              variant={isVideoOn ? "default" : "outline"}
              onClick={toggleVideo}
              className="flex items-center space-x-2"
            >
              {isVideoOn ? (
                <Video className="h-4 w-4" />
              ) : (
                <VideoOff className="h-4 w-4" />
              )}
              <span>Video</span>
            </Button>

            <Button
              variant={isAudioOn ? "default" : "outline"}
              onClick={toggleAudio}
              className="flex items-center space-x-2"
            >
              {isAudioOn ? (
                <Volume2 className="h-4 w-4" />
              ) : (
                <VolumeX className="h-4 w-4" />
              )}
              <span>Audio</span>
            </Button>

            <Button
              variant="outline"
              onClick={() => setIsCommunicationOpen(true)}
              className="flex items-center space-x-2"
            >
              <MessageSquare className="h-4 w-4" />
              <span>Note</span>
            </Button>
          </div>
        </CardContent>
      </Card>

      {/* Quick Actions */}
      <Card>
        <CardHeader>
          <CardTitle className="flex items-center space-x-2">
            <Zap className="h-5 w-5" />
            <span>Quick Actions</span>
          </CardTitle>
        </CardHeader>
        <CardContent>
          <div className="grid grid-cols-1 gap-2">
            {quickActions.map((action) => (
              <Button
                key={action.id}
                variant={
                  action.type === "emergency"
                    ? "destructive"
                    : action.type === "primary"
                      ? "default"
                      : "outline"
                }
                onClick={() => handleQuickAction(action)}
                className="flex h-12 items-center justify-start space-x-3"
              >
                <action.icon className="h-4 w-4 flex-shrink-0" />
                <div className="flex-1 text-left">
                  <h4 className="font-medium">{action.label}</h4>
                  <div className="text-xs opacity-75">{action.description}</div>
                </div>
              </Button>
            ))}
          </div>
        </CardContent>
      </Card>

      {/* Step-Specific Controls */}
      {currentStep && currentStep.type === "wizard_action" && (
        <Card>
          <CardHeader>
            <CardTitle className="flex items-center space-x-2">
              <Target className="h-5 w-5" />
              <span>Step Controls</span>
            </CardTitle>
          </CardHeader>
          <CardContent>
            <div className="space-y-3">
              <div className="text-muted-foreground text-sm">
                Current step:{" "}
                <span className="font-medium">{currentStep.name}</span>
              </div>

              <div className="text-muted-foreground text-xs">
                Use the controls below to execute wizard actions for this step.
              </div>
            </div>
          </CardContent>
        </Card>
      )}

      {/* Communication Dialog */}
      <Dialog open={isCommunicationOpen} onOpenChange={setIsCommunicationOpen}>
        <DialogContent>
          <DialogHeader>
            <DialogTitle>Add Intervention Note</DialogTitle>
            <DialogDescription>
              Record an intervention or observation during the trial.
            </DialogDescription>
          </DialogHeader>
          <div className="space-y-4">
            <div>
              <Label htmlFor="intervention-note">Intervention Note</Label>
              <Textarea
                id="intervention-note"
                value={interventionNote}
                onChange={(e) => setInterventionNote(e.target.value)}
                placeholder="Describe the intervention or observation..."
                className="mt-1"
                rows={4}
              />
            </div>
            <div className="flex items-center space-x-2">
              <Clock className="h-4 w-4" />
              <span className="text-muted-foreground text-sm">
                {new Date().toLocaleTimeString()}
              </span>
            </div>
          </div>
          <DialogFooter>
            <Button
              variant="outline"
              onClick={() => setIsCommunicationOpen(false)}
            >
              Cancel
            </Button>
            <Button
              onClick={handleInterventionSubmit}
              disabled={!interventionNote.trim()}
            >
              Submit Note
            </Button>
          </DialogFooter>
        </DialogContent>
      </Dialog>

      {/* Emergency Action Dialog */}
      <Dialog open={showEmergencyDialog} onOpenChange={setShowEmergencyDialog}>
        <DialogContent>
          <DialogHeader>
            <DialogTitle className="flex items-center space-x-2">
              <AlertTriangle className="h-5 w-5" />
              <span>Emergency Action Required</span>
            </DialogTitle>
            <DialogDescription>
              Select the type of emergency action to perform. This will
              immediately stop or override current robot operations.
            </DialogDescription>
          </DialogHeader>
          <div className="space-y-4">
            <div>
              <Label htmlFor="emergency-select">Emergency Action Type</Label>
              <Select
                value={selectedEmergencyAction}
                onValueChange={setSelectedEmergencyAction}
              >
                <SelectTrigger className="mt-1">
                  <SelectValue placeholder="Select emergency action..." />
                </SelectTrigger>
                <SelectContent>
                  {emergencyActions.map((action) => (
                    <SelectItem key={action.value} value={action.value}>
                      {action.label}
                    </SelectItem>
                  ))}
                </SelectContent>
              </Select>
            </div>
            <div className="rounded-lg border p-3">
              <div className="flex items-start space-x-2">
                <AlertTriangle className="mt-0.5 h-4 w-4 flex-shrink-0" />
                <div className="text-sm">
                  <strong>Warning:</strong> Emergency actions will immediately
                  halt all robot operations and may require manual intervention
                  to resume.
                </div>
              </div>
            </div>
          </div>
          <DialogFooter>
            <Button
              variant="outline"
              onClick={() => {
                setShowEmergencyDialog(false);
                setSelectedEmergencyAction("");
              }}
            >
              Cancel
            </Button>
            <Button
              variant="destructive"
              onClick={handleEmergencyAction}
              disabled={!selectedEmergencyAction}
            >
              Execute Emergency Action
            </Button>
          </DialogFooter>
        </DialogContent>
      </Dialog>
    </div>
  );
}
