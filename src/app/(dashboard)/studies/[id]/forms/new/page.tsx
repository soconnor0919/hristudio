"use client";

import { useState } from "react";
import { useParams, useRouter } from "next/navigation";
import { useSession } from "~/lib/auth-client";
import { notFound } from "next/navigation";
import Link from "next/link";
import {
  FileText,
  ArrowLeft,
  Save,
  LayoutTemplate,
  FileSignature,
  ClipboardList,
  FileQuestion,
} from "lucide-react";
import { useBreadcrumbsEffect } from "~/components/ui/breadcrumb-provider";
import { Button } from "~/components/ui/button";
import { Card, CardContent, CardHeader, CardTitle } from "~/components/ui/card";
import { Input } from "~/components/ui/input";
import { Label } from "~/components/ui/label";
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from "~/components/ui/select";
import { api } from "~/trpc/react";
import { toast } from "sonner";
import type { FormField, FormType } from "~/lib/types/forms";
import { FORM_FIELD_TYPES } from "~/lib/types/forms";
import { FormBuilder } from "~/components/forms/FormBuilder";

const formTypes = [
  { value: "consent", label: "Consent Form", icon: FileSignature, description: "Legal/IRB consent documents" },
  { value: "survey", label: "Survey", icon: ClipboardList, description: "Multi-question questionnaires" },
  { value: "questionnaire", label: "Questionnaire", icon: FileQuestion, description: "Custom data collection forms" },
];

export default function NewFormPage() {
  const params = useParams();
  const router = useRouter();
  const { data: session } = useSession();

  const studyId = typeof params.id === "string" ? params.id : "";

  const [formType, setFormType] = useState<string>("");
  const [title, setTitle] = useState("");
  const [description, setDescription] = useState("");
  const [fields, setFields] = useState<FormField[]>([]);
  const [isSubmitting, setIsSubmitting] = useState(false);

  const { data: study } = api.studies.get.useQuery(
    { id: studyId },
    { enabled: !!studyId },
  );

  const { data: templates } = api.forms.listTemplates.useQuery();

  const createFromTemplate = api.forms.createFromTemplate.useMutation({
    onSuccess: (data) => {
      toast.success("Form created from template!");
      router.push(`/studies/${studyId}/forms/${data.id}`);
    },
    onError: (error) => {
      toast.error("Failed to create from template", { description: error.message });
    },
  });

  const createForm = api.forms.create.useMutation({
    onSuccess: (data) => {
      toast.success("Form created successfully!");
      router.push(`/studies/${studyId}/forms/${data.id}`);
    },
    onError: (error) => {
      toast.error("Failed to create form", { description: error.message });
      setIsSubmitting(false);
    },
  });

  useBreadcrumbsEffect([
    { label: "Dashboard", href: "/dashboard" },
    { label: "Studies", href: "/studies" },
    { label: study?.name ?? "Study", href: `/studies/${studyId}` },
    { label: "Forms", href: `/studies/${studyId}/forms` },
    { label: "Create Form" },
  ]);

  if (!session?.user) {
    return notFound();
  }

  const handleSubmit = (e: React.FormEvent) => {
    e.preventDefault();

    if (!formType || !title) {
      toast.error("Please select a form type and enter a title");
      return;
    }

    setIsSubmitting(true);
    createForm.mutate({
      studyId,
      type: formType as FormType,
      title,
      description,
      fields,
      settings: {},
    });
  };

  return (
    <div className="space-y-6">
      <div className="flex items-center gap-4">
        <Button variant="ghost" size="sm" asChild>
          <Link href={`/studies/${studyId}/forms`}>
            <ArrowLeft className="mr-2 h-4 w-4" />
            Back
          </Link>
        </Button>
      </div>

      <div>
        <h1 className="text-2xl font-bold">Create New Form</h1>
        <p className="text-muted-foreground">Design a consent form, survey, or questionnaire</p>
      </div>

      {templates && templates.length > 0 && (
        <Card>
          <CardHeader>
            <CardTitle className="flex items-center gap-2">
              <LayoutTemplate className="h-5 w-5" />
              Start from Template
            </CardTitle>
          </CardHeader>
          <CardContent>
            <div className="grid gap-3 sm:grid-cols-3">
              {templates.map((template) => {
                const TypeIcon = formTypes.find(t => t.value === template.type)?.icon || FileText;
                return (
                  <button
                    key={template.id}
                    type="button"
                    onClick={() => {
                      createFromTemplate.mutate({
                        studyId,
                        templateId: template.id,
                      });
                    }}
                    disabled={createFromTemplate.isPending}
                    className="flex flex-col items-start rounded-lg border p-4 text-left transition-all hover:bg-muted/50 disabled:opacity-50"
                  >
                    <TypeIcon className="mb-2 h-5 w-5 text-muted-foreground" />
                    <span className="font-medium">{template.templateName}</span>
                    <span className="text-muted-foreground text-xs capitalize">{template.type}</span>
                    <span className="text-muted-foreground text-xs mt-1 line-clamp-2">
                      {template.description}
                    </span>
                  </button>
                );
              })}
            </div>
            <div className="mt-4 text-center text-sm text-muted-foreground">
              Or design from scratch below
            </div>
          </CardContent>
        </Card>
      )}

      <form onSubmit={handleSubmit} className="space-y-6">
        <Card>
          <CardHeader>
            <CardTitle>Form Details</CardTitle>
          </CardHeader>
          <CardContent className="space-y-4">
            <div className="space-y-2">
              <Label>Form Type</Label>
              <div className="grid gap-3 sm:grid-cols-3">
                {formTypes.map((type) => (
                  <button
                    key={type.value}
                    type="button"
                    onClick={() => setFormType(type.value)}
                    className={`flex flex-col items-start rounded-lg border p-4 text-left transition-all hover:bg-muted/50 ${
                      formType === type.value
                        ? "border-primary bg-primary/5 ring-1 ring-primary"
                        : "border-border"
                    }`}
                  >
                    <type.icon className={`mb-2 h-5 w-5 ${formType === type.value ? "text-primary" : "text-muted-foreground"}`} />
                    <span className="font-medium">{type.label}</span>
                    <span className="text-muted-foreground text-xs">{type.description}</span>
                  </button>
                ))}
              </div>
            </div>

            <div className="grid gap-4 sm:grid-cols-2">
              <div className="space-y-2">
                <Label htmlFor="title">Title</Label>
                <Input
                  id="title"
                  value={title}
                  onChange={(e) => setTitle(e.target.value)}
                  placeholder="Enter form title"
                  required
                />
              </div>
              <div className="space-y-2">
                <Label htmlFor="description">Description (optional)</Label>
                <Input
                  id="description"
                  value={description}
                  onChange={(e) => setDescription(e.target.value)}
                  placeholder="Brief description"
                />
              </div>
            </div>
          </CardContent>
        </Card>

        <Card>
          <CardHeader className="flex flex-row items-center justify-between">
            <CardTitle>Form Fields</CardTitle>
            <Select onValueChange={(type) => {
              const newField: FormField = {
                id: crypto.randomUUID(),
                type: type as FormField["type"],
                label: `New ${FORM_FIELD_TYPES.find(f => f.value === type)?.label || "Field"}`,
                required: false,
                options: type === "multiple_choice" ? ["Option 1", "Option 2"] : undefined,
              };
              setFields([...fields, newField]);
            }}>
              <SelectTrigger className="w-[200px]">
                <SelectValue placeholder="Add field..." />
              </SelectTrigger>
              <SelectContent>
                {FORM_FIELD_TYPES.map((type) => (
                  <SelectItem key={type.value} value={type.value}>
                    <span className="mr-2">{type.icon}</span>
                    {type.label}
                  </SelectItem>
                ))}
              </SelectContent>
            </Select>
          </CardHeader>
          <CardContent>
            <FormBuilder fields={fields} onFieldsChange={setFields} />
          </CardContent>
        </Card>

        <div className="flex justify-end gap-3">
          <Button variant="outline" asChild>
            <Link href={`/studies/${studyId}/forms`}>Cancel</Link>
          </Button>
          <Button type="submit" disabled={isSubmitting || !formType || !title}>
            <Save className="mr-2 h-4 w-4" />
            {isSubmitting ? "Creating..." : "Create Form"}
          </Button>
        </div>
      </form>
    </div>
  );
}
