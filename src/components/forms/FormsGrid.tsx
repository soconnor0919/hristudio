"use client";

import { useState, useEffect } from "react";
import { FormCard } from "~/components/forms/FormCard";
import { useToast } from "~/hooks/use-toast";

interface Form {
  id: number;
  title: string;
  location: string;
  studyId: number;
  studyTitle: string;
  participantId: number;
  participantName: string;
  previewLocation: string;
}

export function FormsGrid() {
  const [forms, setForms] = useState<Form[]>([]);
  const { toast } = useToast();

  useEffect(() => {
    fetchForms();
  }, []);

  const fetchForms = async () => {
    try {
      const response = await fetch("/api/forms");
      if (!response.ok) {
        throw new Error("Failed to fetch forms");
      }
      const data = await response.json();
      setForms(data);
    } catch (error) {
      console.error("Error fetching forms:", error);
      toast({
        title: "Error",
        description: "Failed to load forms. Please try again.",
        variant: "destructive",
      });
    }
  };

  const handleDelete = async (formId: number) => {
    try {
      const response = await fetch(`/api/forms/${formId}`, {
        method: "DELETE",
      });
      if (!response.ok) {
        throw new Error("Failed to delete form");
      }
      setForms(forms.filter((form) => form.id !== formId));
      toast({
        title: "Success",
        description: "Form deleted successfully",
      });
    } catch (error) {
      console.error("Error deleting form:", error);
      toast({
        title: "Error",
        description: "Failed to delete form. Please try again.",
        variant: "destructive",
      });
    }
  };

  return (
    <div className="grid grid-cols-1 sm:grid-cols-2 md:grid-cols-3 lg:grid-cols-4 xl:grid-cols-6 gap-4">
      {forms.map((form) => (
        <FormCard key={form.id} form={form} onDelete={handleDelete} />
      ))}
    </div>
  );
}