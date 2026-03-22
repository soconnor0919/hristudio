"use client";

import { useEffect, useState } from "react";
import { useSession } from "~/lib/auth-client";
import { notFound } from "next/navigation";
import {
  FileText,
  Loader2,
  Plus,
  Download,
  Edit2,
  Eye,
  Save,
} from "lucide-react";
import {
  EntityView,
  EntityViewHeader,
  EntityViewSection,
  EmptyState,
} from "~/components/ui/entity-view";
import { useBreadcrumbsEffect } from "~/components/ui/breadcrumb-provider";
import { Button } from "~/components/ui/button";
import { Badge } from "~/components/ui/badge";
import { api } from "~/trpc/react";
import { toast } from "sonner";
import { PageHeader } from "~/components/ui/page-header";
import { useEditor, EditorContent } from "@tiptap/react";
import StarterKit from "@tiptap/starter-kit";
import { Markdown } from "tiptap-markdown";
import { Table } from "@tiptap/extension-table";
import { TableRow } from "@tiptap/extension-table-row";
import { TableCell } from "@tiptap/extension-table-cell";
import { TableHeader } from "@tiptap/extension-table-header";
import {
  Bold,
  Italic,
  List,
  ListOrdered,
  Heading1,
  Heading2,
  Quote,
  Table as TableIcon,
} from "lucide-react";
import { downloadPdfFromHtml } from "~/lib/pdf-generator";

const Toolbar = ({ editor }: { editor: any }) => {
  if (!editor) {
    return null;
  }

  return (
    <div className="border-input flex flex-wrap items-center gap-1 rounded-tl-md rounded-tr-md border bg-transparent p-1">
      <Button
        variant="ghost"
        size="sm"
        onClick={() => editor.chain().focus().toggleBold().run()}
        disabled={!editor.can().chain().focus().toggleBold().run()}
        className={editor.isActive("bold") ? "bg-muted" : ""}
      >
        <Bold className="h-4 w-4" />
      </Button>
      <Button
        variant="ghost"
        size="sm"
        onClick={() => editor.chain().focus().toggleItalic().run()}
        disabled={!editor.can().chain().focus().toggleItalic().run()}
        className={editor.isActive("italic") ? "bg-muted" : ""}
      >
        <Italic className="h-4 w-4" />
      </Button>
      <div className="bg-border mx-1 h-6 w-[1px]" />
      <Button
        variant="ghost"
        size="sm"
        onClick={() => editor.chain().focus().toggleHeading({ level: 1 }).run()}
        className={editor.isActive("heading", { level: 1 }) ? "bg-muted" : ""}
      >
        <Heading1 className="h-4 w-4" />
      </Button>
      <Button
        variant="ghost"
        size="sm"
        onClick={() => editor.chain().focus().toggleHeading({ level: 2 }).run()}
        className={editor.isActive("heading", { level: 2 }) ? "bg-muted" : ""}
      >
        <Heading2 className="h-4 w-4" />
      </Button>
      <div className="bg-border mx-1 h-6 w-[1px]" />
      <Button
        variant="ghost"
        size="sm"
        onClick={() => editor.chain().focus().toggleBulletList().run()}
        className={editor.isActive("bulletList") ? "bg-muted" : ""}
      >
        <List className="h-4 w-4" />
      </Button>
      <Button
        variant="ghost"
        size="sm"
        onClick={() => editor.chain().focus().toggleOrderedList().run()}
        className={editor.isActive("orderedList") ? "bg-muted" : ""}
      >
        <ListOrdered className="h-4 w-4" />
      </Button>
      <Button
        variant="ghost"
        size="sm"
        onClick={() => editor.chain().focus().toggleBlockquote().run()}
        className={editor.isActive("blockquote") ? "bg-muted" : ""}
      >
        <Quote className="h-4 w-4" />
      </Button>
      <div className="bg-border mx-1 h-6 w-[1px]" />
      <Button
        variant="ghost"
        size="sm"
        onClick={() =>
          editor
            .chain()
            .focus()
            .insertTable({ rows: 3, cols: 3, withHeaderRow: true })
            .run()
        }
      >
        <TableIcon className="h-4 w-4" />
      </Button>
    </div>
  );
};

interface StudyFormsPageProps {
  params: Promise<{
    id: string;
  }>;
}

export default function StudyFormsPage({ params }: StudyFormsPageProps) {
  const { data: session } = useSession();
  const utils = api.useUtils();
  const [resolvedParams, setResolvedParams] = useState<{ id: string } | null>(
    null,
  );
  const [editorTarget, setEditorTarget] = useState<string>("");

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

  const { data: activeConsentForm, refetch: refetchConsentForm } =
    api.studies.getActiveConsentForm.useQuery(
      { studyId: resolvedParams?.id ?? "" },
      { enabled: !!resolvedParams?.id },
    );

  // Only sync once when form loads to avoid resetting user edits
  useEffect(() => {
    if (activeConsentForm && !editorTarget) {
      setEditorTarget(activeConsentForm.content);
    }
  }, [activeConsentForm, editorTarget]);

  const editor = useEditor({
    extensions: [
      StarterKit,
      Table.configure({
        resizable: true,
      }),
      TableRow,
      TableHeader,
      TableCell,
      Markdown.configure({
        transformPastedText: true,
      }),
    ],
    content: editorTarget || "",
    immediatelyRender: false,
    onUpdate: ({ editor }) => {
      // @ts-ignore
      setEditorTarget(editor.storage.markdown.getMarkdown());
    },
  });

  // Sync Tiptap when editorTarget is set (e.g., from DB) but make sure not to overwrite active edits
  useEffect(() => {
    if (editor && editorTarget && editor.isEmpty) {
      editor.commands.setContent(editorTarget);
    }
  }, [editorTarget, editor]);

  const generateConsentMutation = api.studies.generateConsentForm.useMutation({
    onSuccess: (data) => {
      toast.success("Default Consent Form Generated!");
      setEditorTarget(data.content);
      editor?.commands.setContent(data.content);
      void refetchConsentForm();
      void utils.studies.getActivity.invalidate({
        studyId: resolvedParams?.id ?? "",
      });
    },
    onError: (error) => {
      toast.error("Error generating consent form", {
        description: error.message,
      });
    },
  });

  const updateConsentMutation = api.studies.updateConsentForm.useMutation({
    onSuccess: () => {
      toast.success("Consent Form Saved Successfully!");
      void refetchConsentForm();
      void utils.studies.getActivity.invalidate({
        studyId: resolvedParams?.id ?? "",
      });
    },
    onError: (error) => {
      toast.error("Error saving consent form", { description: error.message });
    },
  });

  const handleDownloadConsent = async () => {
    if (!activeConsentForm || !study || !editor) return;

    try {
      toast.loading("Generating Document...", { id: "pdf-gen" });
      await downloadPdfFromHtml(editor.getHTML(), {
        filename: `Consent_Form_${study.name.replace(/\s+/g, "_")}_v${activeConsentForm.version}.pdf`,
      });
      toast.success("Document Downloaded Successfully!", { id: "pdf-gen" });
    } catch (error) {
      toast.error("Error generating PDF", { id: "pdf-gen" });
      console.error(error);
    }
  };

  useBreadcrumbsEffect([
    { label: "Dashboard", href: "/dashboard" },
    { label: "Studies", href: "/studies" },
    { label: study?.name ?? "Study", href: `/studies/${resolvedParams?.id}` },
    { label: "Forms" },
  ]);

  if (!session?.user) {
    return notFound();
  }

  if (!study) return <div>Loading...</div>;

  return (
    <EntityView>
      <PageHeader
        title="Study Forms"
        description="Manage consent forms and future questionnaires for this study"
        icon={FileText}
      />

      <div className="grid grid-cols-1 gap-8">
        <EntityViewSection
          title="Consent Document"
          icon="FileText"
          description="Design and manage the consent form that participants must sign before participating in your trials."
          actions={
            <div className="flex gap-2">
              <Button
                variant="outline"
                size="sm"
                onClick={() =>
                  generateConsentMutation.mutate({ studyId: study.id })
                }
                disabled={
                  generateConsentMutation.isPending ||
                  updateConsentMutation.isPending
                }
              >
                {generateConsentMutation.isPending ? (
                  <Loader2 className="mr-2 h-4 w-4 animate-spin" />
                ) : (
                  <Plus className="mr-2 h-4 w-4" />
                )}
                Generate Default Template
              </Button>
              {activeConsentForm && (
                <Button
                  size="sm"
                  onClick={() =>
                    updateConsentMutation.mutate({
                      studyId: study.id,
                      content: editorTarget,
                    })
                  }
                  disabled={
                    updateConsentMutation.isPending ||
                    editorTarget === activeConsentForm.content
                  }
                >
                  {updateConsentMutation.isPending ? (
                    <Loader2 className="mr-2 h-4 w-4 animate-spin" />
                  ) : (
                    <Save className="mr-2 h-4 w-4" />
                  )}
                  Save Changes
                </Button>
              )}
            </div>
          }
        >
          {activeConsentForm ? (
            <div className="space-y-4">
              <div className="flex items-center justify-between">
                <div className="space-y-1">
                  <p className="text-sm leading-none font-medium">
                    {activeConsentForm.title}
                  </p>
                  <p className="text-muted-foreground text-sm">
                    v{activeConsentForm.version} • Status: Active
                  </p>
                </div>
                <div className="flex items-center gap-3">
                  <Button
                    size="sm"
                    variant="ghost"
                    onClick={handleDownloadConsent}
                  >
                    <Download className="mr-2 h-4 w-4" />
                    Download PDF
                  </Button>
                  <Badge
                    variant="outline"
                    className="bg-green-50 text-green-700 hover:bg-green-50"
                  >
                    Active
                  </Badge>
                </div>
              </div>

              <div className="bg-muted/30 border-border flex w-full justify-center overflow-hidden rounded-md border p-8">
                <div className="dark:bg-card ring-border flex w-full max-w-4xl flex-col rounded-sm bg-white shadow-xl ring-1">
                  <div className="border-border bg-muted/50 dark:bg-muted/10 border-b">
                    <Toolbar editor={editor} />
                  </div>
                  <div className="editor-container dark:bg-card min-h-[850px] bg-white px-16 py-20 text-sm">
                    <EditorContent
                      editor={editor}
                      className="prose prose-sm dark:prose-invert h-full max-w-none outline-none focus:outline-none focus-visible:outline-none"
                    />
                  </div>
                </div>
              </div>
            </div>
          ) : (
            <EmptyState
              icon="FileText"
              title="No Consent Form"
              description="Generate a boilerplate consent form for this study to download and collect signatures."
            />
          )}
        </EntityViewSection>
      </div>
    </EntityView>
  );
}
