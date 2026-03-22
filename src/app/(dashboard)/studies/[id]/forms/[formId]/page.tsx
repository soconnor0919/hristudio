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
  Eye,
  Edit2,
  Users,
  CheckCircle,
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
import { Tabs, TabsContent, TabsList, TabsTrigger } from "~/components/ui/tabs";
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

const formTypeIcons = {
  consent: FileSignature,
  survey: ClipboardList,
  questionnaire: FileQuestion,
};

const statusColors = {
  pending: "bg-yellow-100 text-yellow-700",
  completed: "bg-green-100 text-green-700",
  rejected: "bg-red-100 text-red-700",
};

interface FormViewPageProps {
  params: Promise<{
    id: string;
    formId: string;
  }>;
}

export default function FormViewPage({ params }: FormViewPageProps) {
  const { data: session } = useSession();
  const router = useRouter();
  const utils = api.useUtils();
  const [resolvedParams, setResolvedParams] = useState<{ id: string; formId: string } | null>(null);
  const [isEditing, setIsEditing] = useState(false);

  const [title, setTitle] = useState("");
  const [description, setDescription] = useState("");
  const [fields, setFields] = useState<Field[]>([]);

  useEffect(() => {
    const resolveParams = async () => {
      const resolved = await params;
      setResolvedParams(resolved);
    };
    void resolveParams();
  }, [params]);

  const { data: study } = api.studies.get.useQuery(
    { id: resolvedParams?.id ?? "" },
    { enabled: !!resolvedParams?.id },
  );

  const { data: form, isLoading } = api.forms.get.useQuery(
    { id: resolvedParams?.formId ?? "" },
    { enabled: !!resolvedParams?.formId },
  );

  const { data: responsesData } = api.forms.getResponses.useQuery(
    { formId: resolvedParams?.formId ?? "", limit: 50 },
    { enabled: !!resolvedParams?.formId },
  );

  const userRole = (study as any)?.userRole;
  const canManage = userRole === "owner" || userRole === "researcher";

  const updateForm = api.forms.update.useMutation({
    onSuccess: () => {
      toast.success("Form updated successfully!");
      setIsEditing(false);
      void utils.forms.get.invalidate({ id: resolvedParams?.formId });
    },
    onError: (error) => {
      toast.error("Failed to update form", { description: error.message });
    },
  });

  useEffect(() => {
    if (form) {
      setTitle(form.title);
      setDescription(form.description || "");
      setFields((form.fields as Field[]) || []);
    }
  }, [form]);

  useBreadcrumbsEffect([
    { label: "Dashboard", href: "/dashboard" },
    { label: "Studies", href: "/studies" },
    { label: study?.name ?? "Study", href: `/studies/${resolvedParams?.id}` },
    { label: "Forms", href: `/studies/${resolvedParams?.id}/forms` },
    { label: form?.title ?? "Form" },
  ]);

  if (!session?.user) {
    return notFound();
  }

  if (isLoading || !form) return <div>Loading...</div>;

  const TypeIcon = formTypeIcons[form.type as keyof typeof formTypeIcons] || FileText;
  const responses = responsesData?.responses ?? [];

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

  const handleSave = () => {
    updateForm.mutate({
      id: form.id,
      title,
      description,
      fields,
      settings: form.settings as Record<string, any>,
    });
  };

  return (
    <div className="container mx-auto max-w-4xl space-y-6 py-6">
      <div className="flex items-center justify-between">
        <div className="flex items-center gap-4">
          <Button variant="ghost" size="sm" asChild>
            <Link href={`/studies/${resolvedParams?.id}/forms`}>
              <ArrowLeft className="mr-2 h-4 w-4" />
              Back
            </Link>
          </Button>
          <div>
            <div className="flex items-center gap-2">
              <TypeIcon className="h-5 w-5 text-muted-foreground" />
              <h1 className="text-2xl font-bold">{form.title}</h1>
              {form.active && (
                <Badge variant="default" className="text-xs">Active</Badge>
              )}
            </div>
            <p className="text-muted-foreground text-sm capitalize">
              {form.type} • Version {form.version}
            </p>
          </div>
        </div>
        {canManage && (
          <div className="flex gap-2">
            {isEditing ? (
              <>
                <Button variant="outline" onClick={() => setIsEditing(false)}>
                  Cancel
                </Button>
                <Button onClick={handleSave} disabled={updateForm.isPending}>
                  <Save className="mr-2 h-4 w-4" />
                  Save Changes
                </Button>
              </>
            ) : (
              <Button onClick={() => setIsEditing(true)}>
                <Edit2 className="mr-2 h-4 w-4" />
                Edit Form
              </Button>
            )}
          </div>
        )}
      </div>

      <Tabs defaultValue="fields" className="space-y-4">
        <TabsList>
          <TabsTrigger value="fields">Fields</TabsTrigger>
          <TabsTrigger value="preview">Preview</TabsTrigger>
          <TabsTrigger value="responses">
            Responses ({responses.length})
          </TabsTrigger>
        </TabsList>

        <TabsContent value="fields">
          {isEditing ? (
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
                  </div>
                ) : (
                  <div className="space-y-4">
                    {fields.map((field) => (
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
          ) : (
            <Card>
              <CardHeader>
                <CardTitle>Form Fields</CardTitle>
              </CardHeader>
              <CardContent>
                {fields.length === 0 ? (
                  <p className="text-muted-foreground">No fields defined</p>
                ) : (
                  <div className="space-y-3">
                    {fields.map((field, index) => (
                      <div key={field.id} className="flex items-center gap-3 rounded-lg border p-3">
                        <span className="flex h-6 w-6 items-center justify-center rounded-full bg-muted text-xs">
                          {index + 1}
                        </span>
                        <div className="flex-1">
                          <p className="font-medium">{field.label}</p>
                          <p className="text-muted-foreground text-xs">
                            {fieldTypes.find(f => f.value === field.type)?.label}
                            {field.required && " • Required"}
                            {field.type === "multiple_choice" && ` • ${field.options?.length} options`}
                          </p>
                        </div>
                      </div>
                    ))}
                  </div>
                )}
              </CardContent>
            </Card>
          )}
        </TabsContent>

        <TabsContent value="preview">
          <Card>
            <CardHeader>
              <CardTitle>Form Preview</CardTitle>
            </CardHeader>
            <CardContent className="space-y-6">
              <div className="space-y-2">
                <h2 className="text-xl font-semibold">{title}</h2>
                {description && <p className="text-muted-foreground">{description}</p>}
              </div>
              {fields.length === 0 ? (
                <p className="text-muted-foreground">No fields to preview</p>
              ) : (
                <div className="space-y-4">
                  {fields.map((field, index) => (
                    <div key={field.id} className="space-y-2">
                      <Label>
                        {index + 1}. {field.label}
                        {field.required && <span className="text-destructive"> *</span>}
                      </Label>
                      {field.type === "text" && (
                        <Input placeholder="Enter your response..." disabled />
                      )}
                      {field.type === "textarea" && (
                        <Textarea placeholder="Enter your response..." disabled />
                      )}
                      {field.type === "multiple_choice" && (
                        <div className="space-y-2">
                          {field.options?.map((opt, i) => (
                            <label key={i} className="flex items-center gap-2">
                              <input type="radio" disabled /> {opt}
                            </label>
                          ))}
                        </div>
                      )}
                      {field.type === "checkbox" && (
                        <label className="flex items-center gap-2">
                          <input type="checkbox" disabled /> Yes
                        </label>
                      )}
                      {field.type === "yes_no" && (
                        <div className="flex gap-4">
                          <label className="flex items-center gap-2">
                            <input type="radio" disabled /> Yes
                          </label>
                          <label className="flex items-center gap-2">
                            <input type="radio" disabled /> No
                          </label>
                        </div>
                      )}
                      {field.type === "rating" && (
                        <div className="flex gap-2">
                          {Array.from({ length: field.settings?.scale || 5 }, (_, i) => (
                            <button key={i} type="button" className="h-8 w-8 rounded border disabled" disabled>
                              {i + 1}
                            </button>
                          ))}
                        </div>
                      )}
                      {field.type === "date" && (
                        <Input type="date" disabled />
                      )}
                      {field.type === "signature" && (
                        <div className="h-24 rounded border bg-muted/50 flex items-center justify-center text-muted-foreground">
                          Signature pad (disabled in preview)
                        </div>
                      )}
                    </div>
                  ))}
                </div>
              )}
            </CardContent>
          </Card>
        </TabsContent>

        <TabsContent value="responses">
          <Card>
            <CardHeader>
              <CardTitle>Form Responses</CardTitle>
            </CardHeader>
            <CardContent>
              {responses.length === 0 ? (
                <div className="flex flex-col items-center justify-center py-8 text-center text-muted-foreground">
                  <Users className="mb-2 h-8 w-8" />
                  <p>No responses yet</p>
                </div>
              ) : (
                <div className="space-y-4">
                  {responses.map((response) => (
                    <div key={response.id} className="rounded-lg border p-4">
                      <div className="flex items-center justify-between mb-3">
                        <div className="flex items-center gap-2">
                          <Users className="h-4 w-4 text-muted-foreground" />
                          <span className="font-medium">
                            {response.participant?.name || response.participant?.participantCode || "Unknown"}
                          </span>
                        </div>
                        <Badge className={`text-xs ${statusColors[response.status as keyof typeof statusColors]}`}>
                          {response.status}
                        </Badge>
                      </div>
                      <div className="space-y-2 text-sm">
                        {Object.entries(response.responses as Record<string, any>).map(([key, value]) => (
                          <div key={key} className="flex gap-2">
                            <span className="text-muted-foreground">{key}:</span>
                            <span>{String(value)}</span>
                          </div>
                        ))}
                      </div>
                      {response.signedAt && (
                        <div className="mt-2 pt-2 border-t text-xs text-muted-foreground">
                          Signed: {new Date(response.signedAt).toLocaleString()}
                        </div>
                      )}
                    </div>
                  ))}
                </div>
              )}
            </CardContent>
          </Card>
        </TabsContent>
      </Tabs>
    </div>
  );
}