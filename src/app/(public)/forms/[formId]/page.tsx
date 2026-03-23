"use client";

import { useEffect, useState } from "react";
import { useParams, useRouter, useSearchParams } from "next/navigation";
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
import { Textarea } from "~/components/ui/textarea";
import { api } from "~/trpc/react";
import { toast } from "sonner";

interface Field {
  id: string;
  type: string;
  label: string;
  required: boolean;
  options?: string[];
  settings?: Record<string, any>;
}

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
  const [formResponses, setFormResponses] = useState<Record<string, any>>({});
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
  const fields = (form.fields as Field[]) || [];

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

  const updateResponse = (fieldId: string, value: any) => {
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
                    <Label
                      htmlFor={field.id}
                      className={
                        fieldErrors[field.id] ? "text-destructive" : ""
                      }
                    >
                      {index + 1}. {field.label}
                      {field.required && (
                        <span className="text-destructive"> *</span>
                      )}
                    </Label>

                    <div className="mt-2">
                      {field.type === "text" && (
                        <Input
                          id={field.id}
                          value={formResponses[field.id] || ""}
                          onChange={(e) =>
                            updateResponse(field.id, e.target.value)
                          }
                          placeholder="Enter your response..."
                          className={
                            fieldErrors[field.id] ? "border-destructive" : ""
                          }
                        />
                      )}

                      {field.type === "textarea" && (
                        <Textarea
                          id={field.id}
                          value={formResponses[field.id] || ""}
                          onChange={(e) =>
                            updateResponse(field.id, e.target.value)
                          }
                          placeholder="Enter your response..."
                          className={
                            fieldErrors[field.id] ? "border-destructive" : ""
                          }
                        />
                      )}

                      {field.type === "multiple_choice" && (
                        <div
                          className={`mt-2 space-y-2 ${fieldErrors[field.id] ? "border-destructive rounded-md border p-2" : ""}`}
                        >
                          {field.options?.map((opt, i) => (
                            <label
                              key={i}
                              className="flex cursor-pointer items-center gap-2"
                            >
                              <input
                                type="radio"
                                name={field.id}
                                value={opt}
                                checked={formResponses[field.id] === opt}
                                onChange={() => updateResponse(field.id, opt)}
                                className="h-4 w-4"
                              />
                              <span className="text-sm">{opt}</span>
                            </label>
                          ))}
                        </div>
                      )}

                      {field.type === "checkbox" && (
                        <div className="flex items-center gap-2">
                          <input
                            type="checkbox"
                            id={field.id}
                            checked={formResponses[field.id] || false}
                            onChange={(e) =>
                              updateResponse(field.id, e.target.checked)
                            }
                            className="h-4 w-4 rounded border-gray-300"
                          />
                          <Label
                            htmlFor={field.id}
                            className="cursor-pointer font-normal"
                          >
                            Yes, I agree
                          </Label>
                        </div>
                      )}

                      {field.type === "yes_no" && (
                        <div className="mt-2 flex gap-4">
                          <label className="flex cursor-pointer items-center gap-2">
                            <input
                              type="radio"
                              name={field.id}
                              value="yes"
                              checked={formResponses[field.id] === "yes"}
                              onChange={() => updateResponse(field.id, "yes")}
                              className="h-4 w-4"
                            />
                            <span className="text-sm">Yes</span>
                          </label>
                          <label className="flex cursor-pointer items-center gap-2">
                            <input
                              type="radio"
                              name={field.id}
                              value="no"
                              checked={formResponses[field.id] === "no"}
                              onChange={() => updateResponse(field.id, "no")}
                              className="h-4 w-4"
                            />
                            <span className="text-sm">No</span>
                          </label>
                        </div>
                      )}

                      {field.type === "rating" && (
                        <div className="mt-2 flex flex-wrap gap-2">
                          {Array.from(
                            { length: field.settings?.scale || 5 },
                            (_, i) => (
                              <label key={i} className="cursor-pointer">
                                <input
                                  type="radio"
                                  name={field.id}
                                  value={String(i + 1)}
                                  checked={formResponses[field.id] === i + 1}
                                  onChange={() =>
                                    updateResponse(field.id, i + 1)
                                  }
                                  className="peer sr-only"
                                />
                                <span className="hover:bg-muted peer-checked:bg-primary peer-checked:text-primary-foreground flex h-10 w-10 items-center justify-center rounded-full border text-sm font-medium transition-colors">
                                  {i + 1}
                                </span>
                              </label>
                            ),
                          )}
                        </div>
                      )}

                      {field.type === "date" && (
                        <Input
                          type="date"
                          id={field.id}
                          value={formResponses[field.id] || ""}
                          onChange={(e) =>
                            updateResponse(field.id, e.target.value)
                          }
                          className={
                            fieldErrors[field.id] ? "border-destructive" : ""
                          }
                        />
                      )}

                      {field.type === "signature" && (
                        <div className="space-y-2">
                          <Input
                            id={field.id}
                            value={formResponses[field.id] || ""}
                            onChange={(e) =>
                              updateResponse(field.id, e.target.value)
                            }
                            placeholder="Type your full name as signature"
                            className={
                              fieldErrors[field.id] ? "border-destructive" : ""
                            }
                          />
                          <p className="text-muted-foreground text-xs">
                            By entering your name above, you confirm that the
                            information provided is accurate.
                          </p>
                        </div>
                      )}
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
