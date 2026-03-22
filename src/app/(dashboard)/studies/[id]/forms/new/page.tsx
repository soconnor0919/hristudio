"use client";

import { useEffect, useState } from "react";
import { useParams, useRouter } from "next/navigation";
import { useSession } from "~/lib/auth-client";
import { notFound } from "next/navigation";
import Link from "next/link";
import {
  FileText,
  ArrowLeft,
  Plus,
  Trash2,
  GripVertical,
  FileSignature,
  ClipboardList,
  FileQuestion,
  Save,
} from "lucide-react";
import { useBreadcrumbsEffect } from "~/components/ui/breadcrumb-provider";
import { Button } from "~/components/ui/button";
import { Card, CardContent, CardHeader, CardTitle } from "~/components/ui/card";
import { Input } from "~/components/ui/input";
import { Label } from "~/components/ui/label";
import { Textarea } from "~/components/ui/textarea";
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from "~/components/ui/select";
import { Badge } from "~/components/ui/badge";
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

const fieldTypes = [
  { value: "text", label: "Text (short)", icon: "📝" },
  { value: "textarea", label: "Text (long)", icon: "📄" },
  { value: "multiple_choice", label: "Multiple Choice", icon: "☑️" },
  { value: "checkbox", label: "Checkbox", icon: "✅" },
  { value: "rating", label: "Rating Scale", icon: "⭐" },
  { value: "yes_no", label: "Yes/No", icon: "✔️" },
  { value: "date", label: "Date", icon: "📅" },
  { value: "signature", label: "Signature", icon: "✍️" },
];

const formTypes = [
  { value: "consent", label: "Consent Form", icon: FileSignature, description: "Legal/IRB consent documents" },
  { value: "survey", label: "Survey", icon: ClipboardList, description: "Multi-question questionnaires" },
  { value: "questionnaire", label: "Questionnaire", icon: FileQuestion, description: "Custom data collection forms" },
];

export default function NewFormPage() {
  const params = useParams();
  const router = useRouter();
  const { data: session } = useSession();
  const utils = api.useUtils();

  const studyId = typeof params.id === "string" ? params.id : "";

  const [formType, setFormType] = useState<string>("");
  const [title, setTitle] = useState("");
  const [description, setDescription] = useState("");
  const [fields, setFields] = useState<Field[]>([]);
  const [isSubmitting, setIsSubmitting] = useState(false);

  const { data: study } = api.studies.get.useQuery(
    { id: studyId },
    { enabled: !!studyId },
  );

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

  const addField = (type: string) => {
    const newField: Field = {
      id: crypto.randomUUID(),
      type,
      label: `New ${fieldTypes.find(f => f.value === type)?.label || "Field"}`,
      required: false,
      options: type === "multiple_choice" ? ["Option 1", "Option 2"] : undefined,
    };
    setFields([...fields, newField]);
  };

  const removeField = (id: string) => {
    setFields(fields.filter(f => f.id !== id));
  };

  const updateField = (id: string, updates: Partial<Field>) => {
    setFields(fields.map(f => f.id === id ? { ...f, ...updates } : f));
  };

  const handleSubmit = (e: React.FormEvent) => {
    e.preventDefault();

    if (!formType || !title) {
      toast.error("Please select a form type and enter a title");
      return;
    }

    setIsSubmitting(true);
    createForm.mutate({
      studyId,
      type: formType as "consent" | "survey" | "questionnaire",
      title,
      description,
      fields,
      settings: {},
    });
  };

  return (
    <div className="container mx-auto max-w-4xl space-y-6 py-6">
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
            <Select onValueChange={addField}>
              <SelectTrigger className="w-[200px]">
                <SelectValue placeholder="Add field..." />
              </SelectTrigger>
              <SelectContent>
                {fieldTypes.map((type) => (
                  <SelectItem key={type.value} value={type.value}>
                    <span className="mr-2">{type.icon}</span>
                    {type.label}
                  </SelectItem>
                ))}
              </SelectContent>
            </Select>
          </CardHeader>
          <CardContent>
            {fields.length === 0 ? (
              <div className="flex flex-col items-center justify-center py-8 text-center text-muted-foreground">
                <FileText className="mb-2 h-8 w-8" />
                <p>No fields added yet</p>
                <p className="text-sm">Use the dropdown above to add fields</p>
              </div>
            ) : (
              <div className="space-y-4">
                {fields.map((field, index) => (
                  <div
                    key={field.id}
                    className="flex items-start gap-3 rounded-lg border p-4"
                  >
                    <div className="flex cursor-grab items-center text-muted-foreground">
                      <GripVertical className="h-5 w-5" />
                    </div>
                    <div className="flex-1 space-y-3">
                      <div className="flex items-center gap-3">
                        <Badge variant="outline" className="text-xs">
                          {fieldTypes.find(f => f.value === field.type)?.icon}{" "}
                          {fieldTypes.find(f => f.value === field.type)?.label}
                        </Badge>
                        <Input
                          value={field.label}
                          onChange={(e) => updateField(field.id, { label: e.target.value })}
                          placeholder="Field label"
                          className="flex-1"
                        />
                        <label className="flex items-center gap-2 text-sm">
                          <input
                            type="checkbox"
                            checked={field.required}
                            onChange={(e) => updateField(field.id, { required: e.target.checked })}
                            className="rounded border-gray-300"
                          />
                          Required
                        </label>
                      </div>
                      {field.type === "multiple_choice" && (
                        <div className="space-y-2">
                          <Label className="text-xs">Options</Label>
                          {field.options?.map((opt, i) => (
                            <div key={i} className="flex items-center gap-2">
                              <Input
                                value={opt}
                                onChange={(e) => {
                                  const newOptions = [...(field.options || [])];
                                  newOptions[i] = e.target.value;
                                  updateField(field.id, { options: newOptions });
                                }}
                                placeholder={`Option ${i + 1}`}
                                className="flex-1"
                              />
                              <Button
                                type="button"
                                variant="ghost"
                                size="icon"
                                onClick={() => {
                                  const newOptions = field.options?.filter((_, idx) => idx !== i);
                                  updateField(field.id, { options: newOptions });
                                }}
                              >
                                <Trash2 className="h-4 w-4" />
                              </Button>
                            </div>
                          ))}
                          <Button
                            type="button"
                            variant="outline"
                            size="sm"
                            onClick={() => {
                              const newOptions = [...(field.options || []), `Option ${(field.options?.length || 0) + 1}`];
                              updateField(field.id, { options: newOptions });
                            }}
                          >
                            <Plus className="mr-1 h-4 w-4" />
                            Add Option
                          </Button>
                        </div>
                      )}
                      {field.type === "rating" && (
                        <div className="flex items-center gap-2 text-sm text-muted-foreground">
                          <span>Scale:</span>
                          <Select
                            value={field.settings?.scale?.toString() || "5"}
                            onValueChange={(val) => updateField(field.id, { settings: { scale: parseInt(val) } })}
                          >
                            <SelectTrigger className="w-[100px]">
                              <SelectValue />
                            </SelectTrigger>
                            <SelectContent>
                              <SelectItem value="5">1-5</SelectItem>
                              <SelectItem value="7">1-7</SelectItem>
                              <SelectItem value="10">1-10</SelectItem>
                            </SelectContent>
                          </Select>
                        </div>
                      )}
                    </div>
                    <Button
                      type="button"
                      variant="ghost"
                      size="icon"
                      onClick={() => removeField(field.id)}
                    >
                      <Trash2 className="h-4 w-4 text-destructive" />
                    </Button>
                  </div>
                ))}
              </div>
            )}
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