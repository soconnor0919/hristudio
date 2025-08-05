"use client";

import {
    AlertTriangle, Camera, Clock, Hand, HelpCircle, Lightbulb, MessageSquare, Pause,
    Play,
    RotateCcw, Target, Video,
    VideoOff, Volume2,
    VolumeX, Zap
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
    DialogTitle
} from "~/components/ui/dialog";
import { Label } from "~/components/ui/label";
import {
    Select,
    SelectContent,
    SelectItem,
    SelectTrigger,
    SelectValue
} from "~/components/ui/select";
import { Textarea } from "~/components/ui/textarea";

interface ActionControlsProps {
  currentStep: {
    id: string;
    name: string;
    type: "wizard_action" | "robot_action" | "parallel_steps" | "conditional_branch";
    parameters?: any;
    actions?: any[];
  } | null;
  onExecuteAction: (actionType: string, actionData: any) => Promise<void>;
  trialId: string;
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

export function ActionControls({ currentStep, onExecuteAction, trialId }: ActionControlsProps) {
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

  const handleQuickAction = async (action: QuickAction) => {
    if (action.requiresConfirmation) {
      setShowEmergencyDialog(true);
      return;
    }

    try {
      await onExecuteAction(action.action, {
        action_id: action.id,
        step_id: currentStep?.id,
        timestamp: new Date().toISOString(),
      });
    } catch (_error) {
      console.error(`Failed to execute ${action.action}:`, _error);
    }
  };

  const handleEmergencyAction = async () => {
    if (!selectedEmergencyAction) return;

    try {
      await onExecuteAction("emergency_action", {
        emergency_type: selectedEmergencyAction,
        step_id: currentStep?.id,
        timestamp: new Date().toISOString(),
        severity: "high",
      });
      setShowEmergencyDialog(false);
      setSelectedEmergencyAction("");
    } catch (_error) {
      console.error("Failed to execute emergency action:", _error);
    }
  };

  const handleInterventionSubmit = async () => {
    if (!interventionNote.trim()) return;

    try {
      await onExecuteAction("wizard_intervention", {
        intervention_type: "note",
        content: interventionNote,
        step_id: currentStep?.id,
        timestamp: new Date().toISOString(),
      });
      setInterventionNote("");
      setIsCommunicationOpen(false);
    } catch (_error) {
      console.error("Failed to submit intervention:", _error);
    }
  };

  const toggleRecording = async () => {
    const newState = !isRecording;
    setIsRecording(newState);

    await onExecuteAction("recording_control", {
      action: newState ? "start_recording" : "stop_recording",
      timestamp: new Date().toISOString(),
    });
  };

  const toggleVideo = async () => {
    const newState = !isVideoOn;
    setIsVideoOn(newState);

    await onExecuteAction("video_control", {
      action: newState ? "video_on" : "video_off",
      timestamp: new Date().toISOString(),
    });
  };

  const toggleAudio = async () => {
    const newState = !isAudioOn;
    setIsAudioOn(newState);

    await onExecuteAction("audio_control", {
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
              <div className={`w-2 h-2 rounded-full ${isRecording ? "bg-white animate-pulse" : "bg-red-500"}`}></div>
              <span>{isRecording ? "Stop Recording" : "Start Recording"}</span>
            </Button>

            <Button
              variant={isVideoOn ? "default" : "outline"}
              onClick={toggleVideo}
              className="flex items-center space-x-2"
            >
              {isVideoOn ? <Video className="h-4 w-4" /> : <VideoOff className="h-4 w-4" />}
              <span>Video</span>
            </Button>

            <Button
              variant={isAudioOn ? "default" : "outline"}
              onClick={toggleAudio}
              className="flex items-center space-x-2"
            >
              {isAudioOn ? <Volume2 className="h-4 w-4" /> : <VolumeX className="h-4 w-4" />}
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
                  action.type === "emergency" ? "destructive" :
                  action.type === "primary" ? "default" : "outline"
                }
                onClick={() => handleQuickAction(action)}
                className="flex items-center justify-start space-x-3 h-12"
              >
                <action.icon className="h-4 w-4 flex-shrink-0" />
                <div className="flex-1 text-left">
                  <div className="font-medium">{action.label}</div>
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
              <div className="text-sm text-slate-600">
                Current step: <span className="font-medium">{currentStep.name}</span>
              </div>

              {currentStep.actions && currentStep.actions.length > 0 && (
                <div className="space-y-2">
                  <Label className="text-sm font-medium">Available Actions:</Label>
                  <div className="grid gap-2">
                    {currentStep.actions.map((action: any, index: number) => (
                      <Button
                        key={action.id || index}
                        variant="outline"
                        size="sm"
                        onClick={() => onExecuteAction(`step_action_${action.id}`, action)}
                        className="justify-start text-left"
                      >
                        <Play className="h-3 w-3 mr-2" />
                        {action.name}
                      </Button>
                    ))}
                  </div>
                </div>
              )}
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
              <Clock className="h-4 w-4 text-slate-500" />
              <span className="text-sm text-slate-500">
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
            <DialogTitle className="flex items-center space-x-2 text-red-600">
              <AlertTriangle className="h-5 w-5" />
              <span>Emergency Action Required</span>
            </DialogTitle>
            <DialogDescription>
              Select the type of emergency action to perform. This will immediately stop or override current robot operations.
            </DialogDescription>
          </DialogHeader>
          <div className="space-y-4">
            <div>
              <Label htmlFor="emergency-select">Emergency Action Type</Label>
              <Select value={selectedEmergencyAction} onValueChange={setSelectedEmergencyAction}>
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
            <div className="bg-red-50 border border-red-200 rounded-lg p-3">
              <div className="flex items-start space-x-2">
                <AlertTriangle className="h-4 w-4 text-red-600 mt-0.5 flex-shrink-0" />
                <div className="text-sm text-red-800">
                  <strong>Warning:</strong> Emergency actions will immediately halt all robot operations and may require manual intervention to resume.
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
