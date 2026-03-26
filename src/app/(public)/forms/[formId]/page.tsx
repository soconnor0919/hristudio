"use client";

import { useEffect, useState } from "react";
import { useParams, useSearchParams } from "next/navigation";
import Link from "next/link";
import {
  FileText,
  FileSignature,
  ClipboardList,
  FileQuestion,
  CheckCircle,
  AlertCircle,
  Loader2,
} from "lucide-react";
import { Button } from "~/components/ui/button";
import {
  Card,
  CardContent,
  CardHeader,
  CardTitle,
  CardDescription,
} from "~/components/ui/card";
import { Input } from "~/components/ui/input";
import { Label } from "~/components/ui/label";
import { api } from "~/trpc/react";
import { toast } from "sonner";
import type { FormField } from "~/lib/types/forms";
import { FormFieldRenderer } from "~/components/forms/FormFieldRenderer";

const formTypeIcons = {
  consent: FileSignature,
  survey: ClipboardList,
  questionnaire: FileQuestion,
};

export default function ParticipantFormPage() {
  const params = useParams();
  const searchParams = useSearchParams();
  const formId = params.formId as string;

  const [participantCode, setParticipantCode] = useState("");
  const [formResponses, setFormResponses] = useState<Record<string, unknown>>({});
  const [hasSubmitted, setHasSubmitted] = useState(false);
  const [fieldErrors, setFieldErrors] = useState<Record<string, string>>({});

  const { data: form, isLoading: formLoading } = api.forms.getPublic.useQuery(
    { id: formId },
    { enabled: !!formId },
  );

  const submitResponse = api.forms.submitPublic.useMutation({
    onSuccess: () => {
      toast.success("Response submitted successfully!");
      setHasSubmitted(true);
    },
    onError: (error: { message: string }) => {
      toast.error("Submission failed", { description: error.message });
    },
  });

  useEffect(() => {
    const code = searchParams.get("code");
    if (code) {
      setParticipantCode(code);
    }
  }, [searchParams]);

  if (formLoading) {
    return (
      <div className="bg-background flex min-h-[60vh] items-center justify-center">
        <Loader2 className="text-muted-foreground h-8 w-8 animate-spin" />
      </div>
    );
  }

  if (!form) {
    return (
      <div className="bg-background flex min-h-[60vh] flex-col items-center justify-center text-center">
        <AlertCircle className="text-destructive mb-4 h-12 w-12" />
        <h1 className="text-2xl font-bold">Form Not Found</h1>
        <p className="text-muted-foreground mt-2">
          This form may have been removed or the link is invalid.
        </p>
      </div>
    );
  }

  if (hasSubmitted) {
    return (
      <div className="bg-background flex min-h-[60vh] flex-col items-center justify-center text-center">
        <div className="mb-4 rounded-full bg-green-100 p-4">
          <CheckCircle className="h-12 w-12 text-green-600" />
        </div>
        <h1 className="text-2xl font-bold text-green-600">Thank You!</h1>
        <p className="text-muted-foreground mt-2 max-w-md">
          Your response has been submitted successfully.
          {form.type === "consent" && " Please proceed with your session."}
        </p>
        <Button variant="outline" className="mt-6" asChild>
          <Link href="/">Return Home</Link>
        </Button>
      </div>
    );
  }

  const TypeIcon =
    formTypeIcons[form.type as keyof typeof formTypeIcons] || FileText;
  const fields = (form.fields as FormField[]) || [];

  const validateForm = (): boolean => {
    const errors: Record<string, string> = {};
    let isValid = true;

    fields.forEach((field) => {
      if (field.required) {
        const value = formResponses[field.id];
        if (
          value === undefined ||
          value === null ||
          value === "" ||
          (typeof value === "string" && value.trim() === "")
        ) {
          errors[field.id] = "This field is required";
          isValid = false;
        }
      }
    });

    setFieldErrors(errors);
    return isValid;
  };

  const handleSubmit = (e: React.FormEvent) => {
    e.preventDefault();

    if (!participantCode.trim()) {
      toast.error("Please enter your participant code");
      return;
    }

    if (!validateForm()) {
      toast.error("Please fill in all required fields");
      return;
    }

    submitResponse.mutate({
      formId,
      participantCode: participantCode.trim(),
      responses: formResponses,
    });
  };

  const updateResponse = (fieldId: string, value: unknown) => {
    setFormResponses({ ...formResponses, [fieldId]: value });
    if (fieldErrors[fieldId]) {
      const newErrors = { ...fieldErrors };
      delete newErrors[fieldId];
      setFieldErrors(newErrors);
    }
  };

  return (
    <div className="bg-background min-h-screen py-8">
      <div className="mx-auto max-w-2xl px-4">
        <div className="mb-8 text-center">
          <div className="bg-primary/10 mb-4 inline-flex rounded-full p-3">
            <TypeIcon className="text-primary h-8 w-8" />
          </div>
          <h1 className="text-3xl font-bold">{form.title}</h1>
          {form.description && (
            <p className="text-muted-foreground mt-3 text-lg">
              {form.description}
            </p>
          )}
        </div>

        <Card>
          <CardHeader>
            <CardTitle className="text-lg">
              {form.type === "consent"
                ? "Consent Form"
                : form.type === "survey"
                  ? "Survey"
                  : "Questionnaire"}
            </CardTitle>
            <CardDescription>
              Fields marked with <span className="text-destructive">*</span> are
              required
            </CardDescription>
          </CardHeader>
          <CardContent>
            <form onSubmit={handleSubmit} className="space-y-6">
              <div className="space-y-2">
                <Label htmlFor="participantCode">
                  Participant Code <span className="text-destructive">*</span>
                </Label>
                <Input
                  id="participantCode"
                  value={participantCode}
                  onChange={(e) => setParticipantCode(e.target.value)}
                  placeholder="Enter your participant code (e.g., P001)"
                  required
                />
                <p className="text-muted-foreground text-xs">
                  Enter the participant code provided by the researcher
                </p>
              </div>

              <div className="border-t pt-6">
                {fields.map((field, index) => (
                  <div key={field.id} className="mb-6 last:mb-0">
                    <FormFieldLabel
                      field={field}
                      index={index}
                      showIndex
                    />

                    <div className="mt-2">
                      <FormFieldRenderer
                        field={field}
                        value={formResponses[field.id]}
                        onChange={(val) => updateResponse(field.id, val)}
                        mode="participant"
                        index={index}
                        error={fieldErrors[field.id]}
                      />
                    </div>

                    {fieldErrors[field.id] && (
                      <p className="text-destructive mt-1 text-sm">
                        {fieldErrors[field.id]}
                      </p>
                    )}
                  </div>
                ))}
              </div>

              <div className="border-t pt-6">
                <Button
                  type="submit"
                  size="lg"
                  className="w-full"
                  disabled={submitResponse.isPending}
                >
                  {submitResponse.isPending ? (
                    <>
                      <Loader2 className="mr-2 h-4 w-4 animate-spin" />
                      Submitting...
                    </>
                  ) : (
                    <>
                      <CheckCircle className="mr-2 h-4 w-4" />
                      Submit Response
                    </>
                  )}
                </Button>
              </div>
            </form>
          </CardContent>
        </Card>

        <p className="text-muted-foreground mt-6 text-center text-sm">
          Powered by HRIStudio
        </p>
      </div>
    </div>
  );
}

function FormFieldLabel({
  field,
  index,
  showIndex = true,
  error,
}: {
  field: FormField;
  index: number;
  showIndex?: boolean;
  error?: string;
}) {
  return (
    <Label
      className={error ? "text-destructive" : ""}
    >
      {showIndex && `${index + 1}. `}
      {field.label}
      {field.required && <span className="text-destructive"> *</span>}
    </Label>
  );
}
