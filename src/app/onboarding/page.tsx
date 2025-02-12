"use client";

import { useRouter, useSearchParams } from "next/navigation";
import { useSession } from "next-auth/react";
import { api } from "~/trpc/react";
import { Button } from "~/components/ui/button";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "~/components/ui/card";
import { useToast } from "~/hooks/use-toast";
import Link from "next/link";
import { Logo } from "~/components/logo";
import { StudyForm, type StudyFormValues } from "~/components/studies/study-form";
import { useState } from "react";
import { ArrowLeft, ArrowRight, Bot, Users, Microscope, Beaker, GitBranch } from "lucide-react";
import { LucideIcon } from "lucide-react";
import { motion, AnimatePresence } from "framer-motion";

interface OnboardingStep {
  id: string;
  title: string;
  description: string;
  icon: LucideIcon;
  content?: React.ReactNode;
}

// Define the onboarding steps
const ONBOARDING_STEPS: OnboardingStep[] = [
  {
    id: "welcome",
    title: "Welcome to HRIStudio",
    description: "Your platform for human-robot interaction research",
    icon: Bot,
    content: (
      <div className="space-y-4">
        <p className="text-muted-foreground">
          HRIStudio is a comprehensive platform designed to help researchers:
        </p>
        <ul className="list-disc list-inside space-y-2 text-muted-foreground">
          <li>Design and run Wizard-of-Oz experiments</li>
          <li>Manage research participants and data collection</li>
          <li>Collaborate with team members in real-time</li>
          <li>Analyze and export research data</li>
        </ul>
      </div>
    ),
  },
  {
    id: "roles",
    title: "Understanding Roles",
    description: "Different roles for different responsibilities",
    icon: Users,
    content: (
      <div className="space-y-4">
        <p className="text-muted-foreground">
          HRIStudio supports various team roles:
        </p>
        <ul className="list-disc list-inside space-y-2 text-muted-foreground">
          <li><span className="font-medium text-foreground">Owner & Admin:</span> Manage study settings and team</li>
          <li><span className="font-medium text-foreground">Principal Investigator:</span> Oversee research design</li>
          <li><span className="font-medium text-foreground">Wizard:</span> Control robot behavior during experiments</li>
          <li><span className="font-medium text-foreground">Researcher:</span> Analyze data and results</li>
          <li><span className="font-medium text-foreground">Observer:</span> View and annotate sessions</li>
        </ul>
      </div>
    ),
  },
  {
    id: "studies",
    title: "Managing Studies",
    description: "Organize your research effectively",
    icon: Microscope,
    content: (
      <div className="space-y-4">
        <p className="text-muted-foreground">
          Studies are the core of HRIStudio:
        </p>
        <ul className="list-disc list-inside space-y-2 text-muted-foreground">
          <li>Create multiple studies for different research projects</li>
          <li>Invite team members with specific roles</li>
          <li>Manage participant recruitment and data</li>
          <li>Configure experiment protocols and settings</li>
        </ul>
      </div>
    ),
  },
  {
    id: "hierarchy",
    title: "Study Structure",
    description: "Understanding the experiment hierarchy",
    icon: GitBranch,
    content: (
      <div className="space-y-6">
        <div className="relative mx-auto w-full max-w-[400px]">
          {/* Study Level */}
          <motion.div
            initial={{ opacity: 0, y: 20 }}
            animate={{ opacity: 1, y: 0 }}
            transition={{ delay: 0.1 }}
            className="mx-auto mb-4 w-48 rounded-lg border bg-card p-3 text-center shadow-sm"
          >
            <div className="font-medium">Study</div>
            <div className="text-xs text-muted-foreground">Research Project</div>
          </motion.div>

          {/* Connecting Line */}
          <motion.div
            initial={{ opacity: 0 }}
            animate={{ opacity: 1 }}
            transition={{ delay: 0.2 }}
            className="absolute left-1/2 top-[60px] h-8 w-px -translate-x-1/2 bg-border"
          />

          {/* Experiments Level */}
          <motion.div
            initial={{ opacity: 0, y: 20 }}
            animate={{ opacity: 1, y: 0 }}
            transition={{ delay: 0.3 }}
            className="mx-auto mb-4 w-48 rounded-lg border bg-card p-3 text-center shadow-sm"
          >
            <div className="font-medium">Experiments</div>
            <div className="text-xs text-muted-foreground">Study Protocols</div>
          </motion.div>

          {/* Connecting Line */}
          <motion.div
            initial={{ opacity: 0 }}
            animate={{ opacity: 1 }}
            transition={{ delay: 0.4 }}
            className="absolute left-1/2 top-[140px] h-8 w-px -translate-x-1/2 bg-border"
          />

          {/* Trials Level */}
          <motion.div
            initial={{ opacity: 0, y: 20 }}
            animate={{ opacity: 1, y: 0 }}
            transition={{ delay: 0.5 }}
            className="mx-auto mb-4 w-48 rounded-lg border bg-card p-3 text-center shadow-sm"
          >
            <div className="font-medium">Trials</div>
            <div className="text-xs text-muted-foreground">Individual Sessions</div>
          </motion.div>

          {/* Connecting Line */}
          <motion.div
            initial={{ opacity: 0 }}
            animate={{ opacity: 1 }}
            transition={{ delay: 0.6 }}
            className="absolute left-1/2 top-[220px] h-8 w-px -translate-x-1/2 bg-border"
          />

          {/* Steps Level */}
          <motion.div
            initial={{ opacity: 0, y: 20 }}
            animate={{ opacity: 1, y: 0 }}
            transition={{ delay: 0.7 }}
            className="mx-auto mb-4 w-48 rounded-lg border bg-card p-3 text-center shadow-sm"
          >
            <div className="font-medium">Steps</div>
            <div className="text-xs text-muted-foreground">Trial Procedures</div>
          </motion.div>

          {/* Connecting Line */}
          <motion.div
            initial={{ opacity: 0 }}
            animate={{ opacity: 1 }}
            transition={{ delay: 0.8 }}
            className="absolute left-1/2 bottom-[60px] h-8 w-px -translate-x-1/2 bg-border"
          />

          {/* Actions Level */}
          <motion.div
            initial={{ opacity: 0, y: 20 }}
            animate={{ opacity: 1, y: 0 }}
            transition={{ delay: 0.9 }}
            className="mx-auto w-48 rounded-lg border bg-card p-3 text-center shadow-sm"
          >
            <div className="font-medium">Actions</div>
            <div className="text-xs text-muted-foreground">Individual Operations</div>
          </motion.div>
        </div>

        <div className="text-sm text-muted-foreground">
          <p>The experiment structure flows from top to bottom:</p>
          <ul className="mt-2 list-inside list-disc space-y-1">
            <li><span className="font-medium text-foreground">Study:</span> Contains experiments and team members</li>
            <li><span className="font-medium text-foreground">Experiments:</span> Define reusable protocols</li>
            <li><span className="font-medium text-foreground">Trials:</span> Individual sessions with participants</li>
            <li><span className="font-medium text-foreground">Steps:</span> Ordered procedures within a trial</li>
            <li><span className="font-medium text-foreground">Actions:</span> Specific operations (movement, speech, etc.)</li>
          </ul>
        </div>
      </div>
    ),
  },
  {
    id: "experiments",
    title: "Running Experiments",
    description: "Conduct Wizard-of-Oz studies seamlessly",
    icon: Beaker,
    content: (
      <div className="space-y-4">
        <p className="text-muted-foreground">
          Design and execute experiments with ease:
        </p>
        <ul className="list-disc list-inside space-y-2 text-muted-foreground">
          <li>Create reusable experiment templates</li>
          <li>Define robot behaviors and interactions</li>
          <li>Record and annotate sessions in real-time</li>
          <li>Collect and analyze participant data</li>
        </ul>
      </div>
    ),
  },
  {
    id: "setup",
    title: "Let's Get Started",
    description: "Create your first study or join an existing one",
    icon: Bot,
  },
];

// Update slideVariants
const slideVariants = {
  enter: (direction: number) => ({
    x: direction > 0 ? 50 : -50,
    opacity: 0
  }),
  center: {
    zIndex: 1,
    x: 0,
    opacity: 1
  },
  exit: (direction: number) => ({
    zIndex: 0,
    x: direction < 0 ? 50 : -50,
    opacity: 0
  })
};

export default function OnboardingPage() {
  const router = useRouter();
  const searchParams = useSearchParams();
  const { data: session, status } = useSession();
  const { toast } = useToast();
  const [currentStep, setCurrentStep] = useState(0);
  const [direction, setDirection] = useState(0);

  // Get invitation token if it exists
  const token = searchParams.get("token");

  // Fetch invitation if token exists
  const { data: invitation } = api.study.getInvitation.useQuery(
    { token: token! },
    {
      enabled: !!token && status === "authenticated",
      retry: false,
    }
  );

  // Mutation for accepting invitation
  const { mutate: acceptInvitation, isPending: isAccepting } = api.study.acceptInvitation.useMutation({
    onSuccess: () => {
      toast({
        title: "Success",
        description: "You have successfully joined the study.",
      });
      router.push(`/dashboard/studies/${invitation?.studyId}`);
      router.refresh();
    },
    onError: (error) => {
      toast({
        title: "Error",
        description: error.message,
        variant: "destructive",
      });
    },
  });

  // Mutation for creating a new study
  const { mutate: createStudy, isPending: isCreating } = api.study.create.useMutation({
    onSuccess: (data) => {
      toast({
        title: "Success",
        description: "Your study has been created successfully.",
      });
      router.push(`/dashboard/studies/${data.id}`);
      router.refresh();
    },
    onError: (error) => {
      toast({
        title: "Error",
        description: error.message,
        variant: "destructive",
      });
    },
  });

  // Handle study creation
  function onCreateStudy(data: StudyFormValues) {
    createStudy(data);
  }

  // Navigation functions
  const nextStep = () => {
    if (currentStep < ONBOARDING_STEPS.length - 1) {
      setDirection(1);
      setCurrentStep(currentStep + 1);
    }
  };

  const prevStep = () => {
    if (currentStep > 0) {
      setDirection(-1);
      setCurrentStep(currentStep - 1);
    }
  };

  // Ensure currentStep is within bounds
  const safeStep = Math.min(Math.max(0, currentStep), ONBOARDING_STEPS.length - 1);
  const currentStepData = ONBOARDING_STEPS[safeStep]!;
  const Icon = currentStepData.icon;

  // Show loading state while checking authentication
  if (status === "loading") {
    return (
      <div className="auth-gradient relative flex min-h-screen items-center justify-center px-4">
        <Logo 
          className="absolute left-4 top-4 text-lg transition-colors hover:text-primary md:left-8 md:top-8"
          iconClassName="text-primary"
        />
        <div className="w-full max-w-[800px] px-4 py-8">
          <Card className="auth-card shadow-xl transition-shadow hover:shadow-lg">
            <CardContent className="grid p-0 md:grid-cols-2">
              <div className="p-6 md:p-8">
                <div className="mb-6 space-y-2">
                  <CardTitle className="text-2xl font-bold tracking-tight">
                    Loading...
                  </CardTitle>
                  <CardDescription className="text-base">
                    Please wait while we set up your account.
                  </CardDescription>
                </div>
              </div>
              <div className="relative hidden h-full md:block">
                <div className="absolute inset-0 bg-gradient-to-br from-primary/30 via-primary/20 to-secondary/10 rounded-r-lg" />
                <div className="absolute inset-0 flex items-center justify-center">
                  <Logo 
                    className="pointer-events-none"
                    iconClassName="h-32 w-32 mr-0 text-primary/40"
                    textClassName="sr-only"
                  />
                </div>
              </div>
            </CardContent>
          </Card>
        </div>
      </div>
    );
  }

  // Redirect to sign in if not authenticated
  if (status === "unauthenticated") {
    router.push("/auth/signin");
    return null;
  }

  // If user has an invitation and we're on the final step
  if (token && invitation && safeStep === ONBOARDING_STEPS.length - 1) {
    return (
      <div className="auth-gradient relative flex min-h-screen items-center justify-center px-4">
        <Logo 
          className="absolute left-4 top-4 text-lg transition-colors hover:text-primary md:left-8 md:top-8"
          iconClassName="text-primary"
        />
        <div className="w-full max-w-[800px] px-4 py-8">
          <Card className="auth-card shadow-xl transition-shadow hover:shadow-lg">
            <CardContent className="grid p-0 md:grid-cols-2">
              <div className="p-6 md:p-8">
                <div className="mb-6 space-y-2">
                  <CardTitle className="text-2xl font-bold tracking-tight">
                    Join {invitation.study.title}
                  </CardTitle>
                  <CardDescription className="text-base">
                    You've been invited to join as a {invitation.role}.
                  </CardDescription>
                </div>
                <div className="space-y-4">
                  {session?.user.email === invitation.email ? (
                    <Button
                      className="w-full"
                      onClick={() => acceptInvitation({ token })}
                      disabled={isAccepting}
                    >
                      {isAccepting ? "Joining Study..." : "Join Study"}
                    </Button>
                  ) : (
                    <div className="space-y-4">
                      <p className="text-sm text-muted-foreground">
                        This invitation was sent to {invitation.email}, but you're signed in with a different
                        email address ({session?.user.email}).
                      </p>
                      <Button 
                        variant="outline" 
                        className="w-full"
                        onClick={() => router.push("/auth/signin")}
                      >
                        Sign in with correct account
                      </Button>
                    </div>
                  )}
                </div>
              </div>
              <div className="relative hidden h-full md:block">
                <div className="absolute inset-0 bg-gradient-to-br from-primary/30 via-primary/20 to-secondary/10 rounded-r-lg" />
                <div className="absolute inset-0 flex items-center justify-center">
                  <Logo 
                    className="pointer-events-none"
                    iconClassName="h-32 w-32 mr-0 text-primary/40"
                    textClassName="sr-only"
                  />
                </div>
              </div>
            </CardContent>
          </Card>
        </div>
      </div>
    );
  }

  return (
    <div className="auth-gradient relative flex min-h-screen items-center justify-center px-4">
      <Logo 
        className="absolute left-4 top-4 text-lg transition-colors hover:text-primary md:left-8 md:top-8"
        iconClassName="text-primary"
      />
      <div className="w-full max-w-[1000px] px-4 py-8">
        <Card className="auth-card shadow-xl transition-shadow hover:shadow-lg">
          <CardContent className="grid p-0 md:grid-cols-2">
            <div className="relative p-6 md:p-8">
              <div className="mb-6 space-y-2">
                <motion.div 
                  className="mb-8 flex h-12 w-12 items-center justify-center rounded-lg bg-primary/10"
                  whileHover={{ scale: 1.05 }}
                  whileTap={{ scale: 0.95 }}
                >
                  <Icon className="h-6 w-6 text-primary" />
                </motion.div>
                <CardTitle className="text-2xl font-bold tracking-tight">
                  {currentStepData.title}
                </CardTitle>
                <CardDescription className="text-base">
                  {currentStepData.description}
                </CardDescription>
              </div>
              
              <div className="relative h-[280px]">
                <AnimatePresence mode="wait" custom={direction}>
                  <motion.div
                    key={currentStepData.id}
                    custom={direction}
                    variants={slideVariants}
                    initial="enter"
                    animate="center"
                    exit="exit"
                    transition={{
                      x: { type: "spring", stiffness: 300, damping: 30 },
                      opacity: { duration: 0.2 }
                    }}
                    className="absolute inset-0"
                  >
                    <div className="relative h-full">
                      <div className="h-full overflow-y-auto pr-4 scrollbar-thin scrollbar-track-transparent scrollbar-thumb-muted-foreground/20 hover:scrollbar-thumb-muted-foreground/30">
                        {safeStep === ONBOARDING_STEPS.length - 1 ? (
                          <StudyForm
                            defaultValues={{ title: "", description: "" }}
                            onSubmit={onCreateStudy}
                            isSubmitting={isCreating}
                            submitLabel="Create Study"
                          />
                        ) : (
                          <div className="space-y-6">
                            {currentStepData.content}
                          </div>
                        )}
                      </div>
                    </div>
                  </motion.div>
                </AnimatePresence>
              </div>

              <div className="mt-6 flex justify-between pt-4">
                <motion.div whileHover={{ scale: 1.02 }} whileTap={{ scale: 0.98 }}>
                  <Button
                    variant="outline"
                    onClick={prevStep}
                    disabled={safeStep === 0}
                  >
                    <ArrowLeft className="mr-2 h-4 w-4" />
                    Back
                  </Button>
                </motion.div>
                <motion.div whileHover={{ scale: 1.02 }} whileTap={{ scale: 0.98 }}>
                  <Button onClick={nextStep}>
                    Next
                    <ArrowRight className="ml-2 h-4 w-4" />
                  </Button>
                </motion.div>
              </div>
            </div>

            <div className="relative hidden h-full md:block">
              <div className="absolute inset-0 bg-gradient-to-br from-primary/30 via-primary/20 to-secondary/10 rounded-r-lg" />
              <div className="absolute inset-0 flex items-center justify-center">
                <Logo 
                  className="pointer-events-none"
                  iconClassName="h-32 w-32 mr-0 text-primary/40"
                  textClassName="sr-only"
                />
              </div>
              <div className="absolute bottom-8 left-8 right-8">
                <div className="flex justify-between gap-2">
                  {ONBOARDING_STEPS.map((step, index) => (
                    <div
                      key={step.id}
                      className={`h-1 flex-1 rounded-full transition-colors ${
                        index <= safeStep ? "bg-primary" : "bg-border"
                      }`}
                    />
                  ))}
                </div>
              </div>
            </div>
          </CardContent>
        </Card>
      </div>
    </div>
  );
} 